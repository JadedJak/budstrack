#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# BudsTrack - Galaxy Buds3 Pro head tracking for Linux games.
#
# Reads the Buds' spatial-audio orientation stream (the "GRV" quaternion
# Samsung uses for 360 Audio) over the Buds' RFCOMM/SPP channel and turns it
# into game input:
#   * opentrack UDP   (6 little-endian doubles to 127.0.0.1:4242)
#   * virtual joystick (uinput, 3 axes: yaw / pitch / roll)
#   * mouse look       (uinput relative mouse, head delta -> mouse delta)
#
# Protocol details come from GalaxyBudsClient (timschneeb, GPL-3.0); this is
# an independent re-implementation. The calibration / head-frame maths follow
# the approach MousePods (starman-tech, Apache-2.0) uses for AirPods.
#
# Only system Fedora packages are needed: python3-dbus, python3-gobject,
# python3-pyside6 (GUI only).

import argparse
import fcntl
import json
import math
import os
import queue
import socket
import struct
import sys
import threading
import time

APP = "budstrack"
CONFIG_PATH = os.path.join(
    os.environ.get("XDG_CONFIG_HOME", os.path.expanduser("~/.config")), APP, "config.json")
CONTROL_PORT = 4243          # UDP control: "recenter", "toggle", "quit"

SPP_UUID = "2e73a4ad-332d-41fc-90e2-16bef06523f2"   # Buds2 Pro / Buds3 / Buds3 Pro SPP

# ---- Samsung SPP framing -----------------------------------------------------
SOM, EOM = 0xFD, 0xDD
MSG_SET_SPATIAL_AUDIO = 0x7C
MSG_SPATIAL_AUDIO_DATA = 0xC2
MSG_SPATIAL_AUDIO_CONTROL = 0xC3
CTL_ATTACH, CTL_DETACH, CTL_ATTACH_OK, CTL_DETACH_OK, CTL_KEEPALIVE = 0, 1, 2, 3, 4
DATA_GRV, DATA_WEAR_ON, DATA_WEAR_OFF, DATA_GYROCAL, DATA_STUCK = 0x20, 0x21, 0x22, 0x23, 0x24


def crc16(data):
    """CRC-16/XMODEM (poly 0x1021, init 0), as used by the Buds."""
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode(msg_id, payload=b""):
    body = bytes([msg_id]) + bytes(payload)
    header = (len(body) + 2) & 0x3FF            # id + payload + crc; request type = 0
    return (bytes([SOM]) + struct.pack("<H", header) + body
            + struct.pack("<H", crc16(body)) + bytes([EOM]))


class FrameParser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            start = self.buf.find(bytes([SOM]))
            if start < 0:
                self.buf.clear()
                break
            if start:
                del self.buf[:start]
            if len(self.buf) < 3:
                break
            header = struct.unpack_from("<H", self.buf, 1)[0]
            size = header & 0x3FF
            total = 3 + size + 1
            if size < 3:
                del self.buf[:1]
                continue
            if len(self.buf) < total:
                break
            frame = bytes(self.buf[:total])
            if frame[-1] != EOM:
                del self.buf[:1]                 # resync
                continue
            del self.buf[:total]
            body, crc = frame[3:3 + size - 2], struct.unpack_from("<H", frame, 3 + size - 2)[0]
            if crc16(body) != crc:
                continue
            out.append((body[0], body[1:]))
        return out


# ---- small vector / quaternion helpers ------------------------------------
def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def qnorm(q):
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q) if n > 1e-6 else None


def qrot(q, v):
    x, y, z, _ = qmul(qmul(q, (v[0], v[1], v[2], 0.0)), qconj(q))
    return (x, y, z)


def qmat(q):
    x, y, z, w = q
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def vnorm(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v) if n > 1e-9 else None


def vcross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def mmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def mT(a):
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def rotvec(q):
    """Axis * angle (radians) of a unit quaternion."""
    if q[3] < 0:
        q = tuple(-c for c in q)
    s = math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2)
    if s < 1e-9:
        return (0.0, 0.0, 0.0)
    ang = 2 * math.atan2(s, q[3])
    return tuple(c / s * ang for c in q[:3])


UP = (0.0, 0.0, 1.0)     # GRV world frame: Z is vertical (Android convention)


# ---- configuration -----------------------------------------------------------
DEFAULTS = {
    "device": "",                 # Buds MAC; empty = first paired device named "*Buds3 Pro*"
    "lat_body": None,             # head right-axis in earbud frame, learned by calibration
    "output": "opentrack",        # opentrack | joystick | mouse | none
    "opentrack_host": "127.0.0.1",
    "opentrack_port": 4242,
    "invert_yaw": False, "invert_pitch": False, "invert_roll": False,
    "smoothing": 0.5,             # 0 = raw, 1 = heavy
    "deadzone_deg": 0.5,
    "joy_range_yaw": 90.0, "joy_range_pitch": 60.0, "joy_range_roll": 60.0,
    "mouse_px_per_deg": 25.0,
    "mouse_pitch": True,
    "rate_hz": 120,
}


def load_config():
    cfg = dict(DEFAULTS)
    try:
        with open(CONFIG_PATH) as f:
            cfg.update(json.load(f))
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f"config: {e}", file=sys.stderr)
    return cfg


def save_config(cfg):
    os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
    tmp = CONFIG_PATH + ".tmp"
    with open(tmp, "w") as f:
        json.dump(cfg, f, indent=2)
    os.replace(tmp, CONFIG_PATH)


# ---- BlueZ connection --------------------------------------------------------
class BudsLink:
    """Gets an RFCOMM fd for the Buds' SPP service via a BlueZ Profile1, then
    runs attach / keep-alive / read on it. Calls on_quat(q) for each sample."""

    def __init__(self, mac, on_quat, on_status, verbose=False):
        self.mac, self.on_quat, self.on_status, self.verbose = mac, on_quat, on_status, verbose
        self.sock = None
        self.stop_evt = threading.Event()
        self.fd_q = queue.Queue()
        self.loop = None
        self.samples = 0

    # D-Bus side runs in its own GLib loop thread.
    def _dbus_thread(self):
        import dbus
        import dbus.service
        import dbus.mainloop.glib
        from gi.repository import GLib
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        bus = dbus.SystemBus()
        link = self

        class Profile(dbus.service.Object):
            @dbus.service.method("org.bluez.Profile1", in_signature="oha{sv}", out_signature="")
            def NewConnection(self, device, fd, props):
                link.fd_q.put(fd.take())

            @dbus.service.method("org.bluez.Profile1", in_signature="o", out_signature="")
            def RequestDisconnection(self, device):
                link.on_status("Buds asked to disconnect")
                link._close()

            @dbus.service.method("org.bluez.Profile1", in_signature="", out_signature="")
            def Release(self):
                pass

        path = "/org/budstrack/profile"
        self._profile = Profile(bus, path)
        mgr = dbus.Interface(bus.get_object("org.bluez", "/org/bluez"), "org.bluez.ProfileManager1")
        try:
            mgr.RegisterProfile(path, SPP_UUID, {"Name": "BudsTrack", "Role": "client",
                                                  "AutoConnect": dbus.Boolean(False)})
        except dbus.exceptions.DBusException as e:
            if "AlreadyExists" not in str(e):
                self.fd_q.put(e)
                return
        self._bus, self._mgr = bus, mgr
        self.loop = GLib.MainLoop()
        self.fd_q.put("ready")
        self.loop.run()

    def _device_path(self):
        import dbus
        om = dbus.Interface(self._bus.get_object("org.bluez", "/"), "org.freedesktop.DBus.ObjectManager")
        cands = []
        for path, ifs in om.GetManagedObjects().items():
            d = ifs.get("org.bluez.Device1")
            if not d:
                continue
            addr, name = str(d.get("Address", "")), str(d.get("Alias", d.get("Name", "")))
            uuids = [str(u).lower() for u in d.get("UUIDs", [])]
            if self.mac:
                if addr.upper() == self.mac.upper():
                    return path, name, addr
            elif SPP_UUID in uuids or "buds" in name.lower():
                cands.append((bool(d.get("Connected")), path, name, addr))
        if not cands:
            raise RuntimeError("No paired Galaxy Buds found - pair them in KDE Bluetooth settings first")
        cands.sort(reverse=True)
        return cands[0][1:]

    def run(self):
        threading.Thread(target=self._dbus_thread, daemon=True).start()
        r = self.fd_q.get()
        if isinstance(r, Exception):
            raise r
        while not self.stop_evt.is_set():
            try:
                self._session()
            except Exception as e:
                self.on_status(f"Error: {e}")
            self._close()
            if self.stop_evt.wait(3):
                break
            self.on_status("Reconnecting...")
        self.shutdown()

    def _session(self):
        import dbus
        path, name, addr = self._device_path()
        self.on_status(f"Connecting to {name} ({addr})...")
        from gi.repository import GLib
        dev = dbus.Interface(self._bus.get_object("org.bluez", path), "org.bluez.Device1")
        while not self.fd_q.empty():
            self.fd_q.get_nowait()

        def on_err(e):
            if "InProgress" not in str(e) and "AlreadyConnected" not in str(e):
                self.fd_q.put(RuntimeError(f"ConnectProfile failed: {e.get_dbus_message()}"))

        # Call asynchronously from the GLib thread so NewConnection can be
        # dispatched while BlueZ waits for it.
        GLib.idle_add(lambda: dev.ConnectProfile(SPP_UUID, reply_handler=lambda: None,
                                                 error_handler=on_err, timeout=25) and False)
        try:
            fd = self.fd_q.get(timeout=25)
        except queue.Empty:
            raise RuntimeError("timed out waiting for the Buds' data channel")
        if isinstance(fd, Exception):
            raise fd
        self.sock = socket.socket(fileno=fd)
        self.sock.settimeout(1.0)
        self.on_status(f"Connected to {name} - starting head tracking")
        self.send(MSG_SET_SPATIAL_AUDIO, b"\x01")
        self.send(MSG_SPATIAL_AUDIO_CONTROL, bytes([CTL_ATTACH]))
        parser = FrameParser()
        last_ka = time.monotonic()
        last_sample = time.monotonic()
        while not self.stop_evt.is_set():
            now = time.monotonic()
            if now - last_ka >= 2.0:
                self.send(MSG_SPATIAL_AUDIO_CONTROL, bytes([CTL_KEEPALIVE]))
                last_ka = now
            if now - last_sample > 6.0:
                # No data: re-attach (Buds may have been out of ears / in case).
                self.send(MSG_SET_SPATIAL_AUDIO, b"\x01")
                self.send(MSG_SPATIAL_AUDIO_CONTROL, bytes([CTL_ATTACH]))
                last_sample = now
            try:
                data = self.sock.recv(4096)
            except socket.timeout:
                continue
            if not data:
                raise RuntimeError("Buds closed the connection")
            for mid, pl in parser.feed(data):
                if self.verbose and mid != MSG_SPATIAL_AUDIO_DATA:
                    print(f"rx id=0x{mid:02X} {pl.hex()}", file=sys.stderr)
                if mid == MSG_SPATIAL_AUDIO_CONTROL and pl:
                    if pl[0] == CTL_ATTACH_OK:
                        self.on_status(f"Tracking ({name})")
                elif mid == MSG_SPATIAL_AUDIO_DATA and pl:
                    ev = pl[0]
                    if ev == DATA_GRV and len(pl) >= 9:
                        # Wire order is w, x, y, z (verified on a Buds3 Pro:
                        # head turns rotate about world +Z only in this order).
                        w, x, y, z = struct.unpack_from("<4h", pl, 1)
                        q = qnorm((x / 10000.0, y / 10000.0, z / 10000.0, w / 10000.0))
                        if q:
                            last_sample = time.monotonic()
                            self.samples += 1
                            self.on_quat(q)
                    elif ev == DATA_WEAR_OFF:
                        self.on_status("Buds report: not worn")
                    elif ev == DATA_WEAR_ON:
                        self.on_status(f"Tracking ({name})")
                    elif ev == DATA_STUCK:
                        self.on_status("Buds report: sensor stuck - take them out and back in")
                    elif self.verbose:
                        print(f"spatial data 0x{ev:02X} {pl[1:].hex()}", file=sys.stderr)

    def send(self, mid, payload):
        if self.sock:
            self.sock.sendall(encode(mid, payload))

    def _close(self):
        s, self.sock = self.sock, None
        if s:
            try:
                s.sendall(encode(MSG_SPATIAL_AUDIO_CONTROL, bytes([CTL_DETACH])))
                s.sendall(encode(MSG_SET_SPATIAL_AUDIO, b"\x00"))
            except OSError:
                pass
            try:
                s.close()
            except OSError:
                pass

    def stop(self):
        self.stop_evt.set()

    def shutdown(self):
        self._close()
        try:
            self._mgr.UnregisterProfile("/org/budstrack/profile")
        except Exception:
            pass
        if self.loop:
            self.loop.quit()


# ---- head pose -----------------------------------------------------------------
class HeadPose:
    """Turns earbud orientation into head yaw / pitch / roll (degrees).
    yaw + = turn left, pitch + = look up, roll + = tilt right (before inverts)."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.lock = threading.Lock()
        self.q = None
        self.ref = None
        self.E = None
        self.t_last = 0.0
        self.calib = None           # None | dict during nod calibration

    def feed(self, q):
        with self.lock:
            if self.q is not None and sum(a * b for a, b in zip(q, self.q)) < 0:
                q = tuple(-c for c in q)            # keep hemisphere continuous
            self.q = q
            self.t_last = time.monotonic()
            if self.ref is None:
                self._recenter()
            if self.calib is not None:
                self._calib_step()

    def recenter(self):
        with self.lock:
            if self.q is not None:
                self._recenter()

    def _recenter(self, ref=None):
        self.ref = ref or self.q
        lat = self.cfg.get("lat_body")
        if not lat:
            self.E = None
            return
        right = qrot(self.ref, lat)
        right = vnorm((right[0], right[1], 0.0)) or (1.0, 0.0, 0.0)
        fwd = vcross(UP, right)
        self.E = tuple(tuple((right, fwd, UP)[j][i] for j in range(3)) for i in range(3))

    # Calibration: look straight ahead, then nod down and hold.
    def start_calibration(self):
        with self.lock:
            self.calib = {"t0": time.monotonic(), "best": 0.0, "axis": None}
            if self.q is not None:
                self.ref = self.q

    def _calib_step(self):
        d = qmul(self.q, qconj(self.ref))                    # world-frame delta
        rv = rotvec(d)
        ang = math.sqrt(sum(c * c for c in rv))
        if ang > self.calib["best"]:
            self.calib["best"], self.calib["axis"] = ang, rv

    def finish_calibration(self):
        with self.lock:
            c, self.calib = self.calib, None
            if not c or c["best"] < math.radians(15):
                return False
            right_world = vnorm(tuple(-v for v in c["axis"]))     # nod down = -rotation about right
            lat_body = qrot(qconj(self.ref), right_world)
            self.cfg["lat_body"] = list(lat_body)
            self._recenter(self.ref)
            return True

    def angles(self):
        """(yaw, pitch, roll) in degrees, or None if no data / not calibrated."""
        with self.lock:
            if self.q is None or self.E is None or time.monotonic() - self.t_last > 1.0:
                return None
            Rd = mmul(qmat(self.q), mT(qmat(self.ref)))
            M = mmul(mmul(mT(self.E), Rd), self.E)
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, M[2][1]))))
        roll = math.degrees(math.atan2(-M[2][0], M[2][2]))
        yaw = math.degrees(math.atan2(-M[0][1], M[1][1]))
        c = self.cfg
        return (-yaw if c["invert_yaw"] else yaw,
                -pitch if c["invert_pitch"] else pitch,
                -roll if c["invert_roll"] else roll)


# ---- outputs -----------------------------------------------------------------
class OpentrackOut:
    def __init__(self, cfg):
        self.addr = (cfg["opentrack_host"], int(cfg["opentrack_port"]))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, yaw, pitch, roll):
        # opentrack "UDP over network": x, y, z (cm), yaw, pitch, roll (deg)
        self.sock.sendto(struct.pack("<6d", 0.0, 0.0, 0.0, yaw, pitch, roll), self.addr)

    def close(self):
        self.sock.close()


EV_SYN, EV_KEY, EV_REL, EV_ABS = 0, 1, 2, 3
UI_SET_EVBIT, UI_SET_KEYBIT, UI_SET_RELBIT, UI_SET_ABSBIT = 0x40045564, 0x40045565, 0x40045566, 0x40045567
UI_DEV_CREATE, UI_DEV_DESTROY = 0x5501, 0x5502


class UInput:
    def __init__(self, name, keys=(), rels=(), abses=None):
        self.fd = os.open("/dev/uinput", os.O_WRONLY | os.O_NONBLOCK)
        absmin, absmax = [0] * 64, [0] * 64
        if keys:
            fcntl.ioctl(self.fd, UI_SET_EVBIT, EV_KEY)
            for k in keys:
                fcntl.ioctl(self.fd, UI_SET_KEYBIT, k)
        if rels:
            fcntl.ioctl(self.fd, UI_SET_EVBIT, EV_REL)
            for r in rels:
                fcntl.ioctl(self.fd, UI_SET_RELBIT, r)
        if abses:
            fcntl.ioctl(self.fd, UI_SET_EVBIT, EV_ABS)
            for a in abses:
                fcntl.ioctl(self.fd, UI_SET_ABSBIT, a)
                absmin[a], absmax[a] = -32767, 32767
        dev = struct.pack("80sHHHHI", name.encode()[:79], 0x06, 0x1209, 0xB0D5, 1, 0)
        dev += struct.pack("64i", *absmax) + struct.pack("64i", *absmin)
        dev += struct.pack("64i", *([0] * 64)) + struct.pack("64i", *([0] * 64))
        os.write(self.fd, dev)
        fcntl.ioctl(self.fd, UI_DEV_CREATE)

    def emit(self, events):
        buf = b"".join(struct.pack("llHHi", 0, 0, t, c, v) for t, c, v in events)
        buf += struct.pack("llHHi", 0, 0, EV_SYN, 0, 0)
        try:
            os.write(self.fd, buf)
        except BlockingIOError:
            pass

    def close(self):
        try:
            fcntl.ioctl(self.fd, UI_DEV_DESTROY)
        finally:
            os.close(self.fd)


class JoystickOut:
    def __init__(self, cfg):
        self.cfg = cfg
        # ABS_X=yaw, ABS_Y=pitch, ABS_Z=roll; one button so SDL/Steam see a joystick.
        self.ui = UInput("BudsTrack head tracker", keys=(0x120,), abses=(0, 1, 2))

    def send(self, yaw, pitch, roll):
        c = self.cfg

        def ax(v, rng):
            return int(max(-1.0, min(1.0, v / rng)) * 32767)
        self.ui.emit([(EV_ABS, 0, ax(-yaw, c["joy_range_yaw"])),     # turn right = +X
                      (EV_ABS, 1, ax(-pitch, c["joy_range_pitch"])),  # look up = -Y
                      (EV_ABS, 2, ax(roll, c["joy_range_roll"]))])

    def close(self):
        self.ui.close()


class MouseOut:
    """Head movement -> relative mouse movement (works for any mouse-look game)."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.ui = UInput("BudsTrack head mouse", keys=(0x110, 0x111), rels=(0, 1))
        self.prev = None
        self.acc = [0.0, 0.0]

    def send(self, yaw, pitch, roll):
        if self.prev is None:
            self.prev = (yaw, pitch)
            return
        dy_ = (yaw - self.prev[0] + 180) % 360 - 180
        dp = pitch - self.prev[1]
        self.prev = (yaw, pitch)
        k = self.cfg["mouse_px_per_deg"]
        self.acc[0] += -dy_ * k
        self.acc[1] += (-dp * k) if self.cfg["mouse_pitch"] else 0.0
        ix, iy = int(self.acc[0]), int(self.acc[1])
        if ix or iy:
            self.acc[0] -= ix
            self.acc[1] -= iy
            self.ui.emit([(EV_REL, 0, ix), (EV_REL, 1, iy)])

    def reset(self):
        self.prev = None

    def close(self):
        self.ui.close()


OUTPUTS = {"opentrack": OpentrackOut, "joystick": JoystickOut, "mouse": MouseOut}


# ---- engine --------------------------------------------------------------------
class Engine:
    def __init__(self, cfg, verbose=False):
        self.cfg = cfg
        self.pose = HeadPose(cfg)
        self.status = "Starting..."
        self.enabled = True
        self.out = None
        self.out_name = None
        self.smoothed = None
        self.last = None
        self.stop_evt = threading.Event()
        self.link = BudsLink(cfg.get("device", ""), self.pose.feed, self._set_status, verbose)
        self._lock = threading.Lock()

    def _set_status(self, s):
        self.status = s
        print(s, file=sys.stderr, flush=True)

    def start(self):
        threading.Thread(target=self._link_run, daemon=True).start()
        threading.Thread(target=self._loop, daemon=True).start()
        threading.Thread(target=self._control, daemon=True).start()

    def _link_run(self):
        try:
            self.link.run()
        except Exception as e:
            self._set_status(f"Bluetooth error: {e}")

    def _control(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.bind(("127.0.0.1", CONTROL_PORT))
        except OSError:
            return
        s.settimeout(0.5)
        while not self.stop_evt.is_set():
            try:
                cmd = s.recv(64).decode().strip()
            except socket.timeout:
                continue
            if cmd == "recenter":
                self.recenter()
            elif cmd == "toggle":
                self.enabled = not self.enabled
            elif cmd == "quit":
                self.stop()

    def recenter(self):
        self.pose.recenter()
        self.smoothed = None
        if isinstance(self.out, MouseOut):
            self.out.reset()

    def set_output(self, name):
        with self._lock:
            if self.out:
                self.out.close()
            self.out, self.out_name = None, name
            if name in OUTPUTS:
                try:
                    self.out = OUTPUTS[name](self.cfg)
                except Exception as e:
                    self._set_status(f"Output '{name}' failed: {e}")

    def _loop(self):
        self.set_output(self.cfg["output"])
        period = 1.0 / max(30, int(self.cfg["rate_hz"]))
        while not self.stop_evt.is_set():
            t0 = time.monotonic()
            a = self.pose.angles()
            if a is not None:
                alpha = 1.0 - 0.9 * max(0.0, min(1.0, self.cfg["smoothing"]))
                if self.smoothed is None:
                    self.smoothed = list(a)
                else:
                    for i in range(3):
                        d = (a[i] - self.smoothed[i] + 180) % 360 - 180
                        if abs(d) > self.cfg["deadzone_deg"] or alpha >= 1.0:
                            self.smoothed[i] += alpha * d
                self.last = tuple(self.smoothed)
                with self._lock:
                    if self.out and self.enabled:
                        try:
                            self.out.send(*self.last)
                        except OSError:
                            pass
            else:
                self.last = None
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def stop(self):
        self.stop_evt.set()
        self.link.stop()
        with self._lock:
            if self.out:
                self.out.close()
                self.out = None


# ---- GUI -------------------------------------------------------------------------
def run_gui(engine, cfg):
    from PySide6 import QtCore, QtWidgets

    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("BudsTrack")
    app.setDesktopFileName("budstrack")
    w = QtWidgets.QWidget()
    w.setWindowTitle("BudsTrack - Galaxy Buds head tracking")
    lay = QtWidgets.QVBoxLayout(w)

    status = QtWidgets.QLabel()
    status.setWordWrap(True)
    lay.addWidget(status)

    form = QtWidgets.QFormLayout()
    lay.addLayout(form)
    bars = {}
    for name in ("Yaw", "Pitch", "Roll"):
        b = QtWidgets.QProgressBar()
        b.setRange(-90, 90)
        b.setFormat("%v°")
        bars[name] = b
        form.addRow(name, b)
    rate = QtWidgets.QLabel("-")
    form.addRow("Samples/s", rate)

    out = QtWidgets.QComboBox()
    for key, label in (("opentrack", "opentrack (UDP 4242)"), ("joystick", "Virtual joystick"),
                       ("mouse", "Mouse look"), ("none", "Off (preview only)")):
        out.addItem(label, key)
    out.setCurrentIndex(max(0, out.findData(cfg["output"])))
    form.addRow("Output", out)

    smooth = QtWidgets.QSlider(QtCore.Qt.Horizontal)
    smooth.setRange(0, 100)
    smooth.setValue(int(cfg["smoothing"] * 100))
    form.addRow("Smoothing", smooth)

    sens = QtWidgets.QDoubleSpinBox()
    sens.setRange(1, 200)
    sens.setValue(cfg["mouse_px_per_deg"])
    sens.setSuffix(" px/°")
    form.addRow("Mouse speed", sens)

    inv = QtWidgets.QHBoxLayout()
    invs = {}
    for k in ("yaw", "pitch", "roll"):
        cb = QtWidgets.QCheckBox(f"Invert {k}")
        cb.setChecked(cfg[f"invert_{k}"])
        invs[k] = cb
        inv.addWidget(cb)
    lay.addLayout(inv)

    btns = QtWidgets.QHBoxLayout()
    b_center = QtWidgets.QPushButton("Recenter")
    b_cal = QtWidgets.QPushButton("Calibrate…")
    b_pause = QtWidgets.QPushButton("Pause output")
    b_pause.setCheckable(True)
    for b in (b_center, b_cal, b_pause):
        btns.addWidget(b)
    lay.addLayout(btns)
    hint = QtWidgets.QLabel(f"Recenter from a KDE shortcut: <code>{os.path.abspath(__file__)} --send recenter</code>")
    hint.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
    hint.setWordWrap(True)
    lay.addWidget(hint)

    def persist():
        cfg["output"] = out.currentData()
        cfg["smoothing"] = smooth.value() / 100.0
        cfg["mouse_px_per_deg"] = sens.value()
        for k, cb in invs.items():
            cfg[f"invert_{k}"] = cb.isChecked()
        save_config(cfg)

    out.currentIndexChanged.connect(lambda _: (engine.set_output(out.currentData()), persist()))
    smooth.valueChanged.connect(lambda _: persist())
    sens.valueChanged.connect(lambda _: persist())
    for cb in invs.values():
        cb.toggled.connect(lambda _: persist())
    b_center.clicked.connect(engine.recenter)
    b_pause.toggled.connect(lambda on: setattr(engine, "enabled", not on))

    def calibrate():
        if engine.pose.q is None:
            QtWidgets.QMessageBox.information(w, "Calibrate", "Wait until the Buds are tracking.")
            return
        QtWidgets.QMessageBox.information(
            w, "Calibrate",
            "Sit normally and look straight at the screen, then click OK.\n\n"
            "Then slowly nod your head DOWN (chin to chest) and hold it there "
            "until this finishes (4 seconds).")
        engine.pose.start_calibration()
        b_cal.setEnabled(False)
        b_cal.setText("Nod down and hold…")

        def done():
            ok = engine.pose.finish_calibration()
            b_cal.setEnabled(True)
            b_cal.setText("Calibrate…")
            if ok:
                save_config(cfg)
                engine.recenter()
                QtWidgets.QMessageBox.information(w, "Calibrate", "Done. Look at the screen and press Recenter.")
            else:
                QtWidgets.QMessageBox.warning(w, "Calibrate", "Didn't see a clear nod (need > 15°). Try again.")
        QtCore.QTimer.singleShot(4000, done)
    b_cal.clicked.connect(calibrate)

    last_n = [0, time.monotonic()]

    def tick():
        s = engine.status
        if engine.pose.q is not None and not cfg.get("lat_body"):
            s += "\nNot calibrated yet: click Calibrate… (one time)."
        status.setText(s)
        a = engine.last
        for i, name in enumerate(("Yaw", "Pitch", "Roll")):
            bars[name].setValue(int(max(-90, min(90, a[i]))) if a else 0)
        now = time.monotonic()
        if now - last_n[1] >= 1.0:
            rate.setText(f"{(engine.link.samples - last_n[0]) / (now - last_n[1]):.0f}")
            last_n[:] = [engine.link.samples, now]
    t = QtCore.QTimer()
    t.timeout.connect(tick)
    t.start(50)

    w.resize(460, 0)
    w.show()
    rc = app.exec()
    engine.stop()
    time.sleep(0.3)
    return rc


def run_cli(engine, cfg, calibrate):
    import signal
    signal.signal(signal.SIGINT, lambda *_: engine.stop())
    signal.signal(signal.SIGTERM, lambda *_: engine.stop())
    if calibrate:
        print("Waiting for data...", file=sys.stderr)
        while engine.pose.q is None and not engine.stop_evt.wait(0.2):
            pass
        input("Look straight ahead and press Enter, then nod DOWN and hold for 4 s...")
        engine.pose.start_calibration()
        time.sleep(4)
        if engine.pose.finish_calibration():
            save_config(cfg)
            print("Calibrated.", file=sys.stderr)
        else:
            print("No clear nod seen (> 15°).", file=sys.stderr)
    while not engine.stop_evt.wait(0.1):
        a = engine.last
        q = engine.pose.q
        if a:
            line = f"yaw {a[0]:7.1f}  pitch {a[1]:7.1f}  roll {a[2]:7.1f}"
        elif q:
            line = "quat " + " ".join(f"{c:+.3f}" for c in q) + "  (not calibrated)"
        else:
            line = "no data"
        print(f"\r{line:70s}", end="", file=sys.stderr, flush=True)
    print(file=sys.stderr)


def main():
    p = argparse.ArgumentParser(description="Galaxy Buds head tracking for Linux games")
    p.add_argument("--cli", action="store_true", help="no GUI, print angles in the terminal")
    p.add_argument("--calibrate", action="store_true", help="(with --cli) run nod calibration")
    p.add_argument("--device", help="Buds Bluetooth MAC (default: first paired Galaxy Buds)")
    p.add_argument("--output", choices=["opentrack", "joystick", "mouse", "none"])
    p.add_argument("--send", choices=["recenter", "toggle", "quit"],
                   help="send a command to the running BudsTrack and exit")
    p.add_argument("-v", "--verbose", action="store_true", help="log non-motion messages")
    args = p.parse_args()

    if args.send:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.sendto(args.send.encode(), ("127.0.0.1", CONTROL_PORT))
        return 0

    cfg = load_config()
    if args.device:
        cfg["device"] = args.device
    if args.output:
        cfg["output"] = args.output
    engine = Engine(cfg, verbose=args.verbose)
    engine.start()
    if args.cli:
        run_cli(engine, cfg, args.calibrate)
        time.sleep(0.3)
        return 0
    return run_gui(engine, cfg)


if __name__ == "__main__":
    sys.exit(main())
