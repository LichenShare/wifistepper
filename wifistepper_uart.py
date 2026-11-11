"""WifiStepper UART master draft for WSX100 stable firmware 002.

Source reference: goodrobotics/wifistepper commit
3627d32739669bb249ec7a8760bd522925041a52.
No background thread: call poll(), sleep(), or a blocking request regularly.
An ACK is handler receipt, NOT guaranteed enqueue success or motion completion.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import IntEnum
import math
import struct
import time

SOURCE_COMMIT = "3627d32739669bb249ec7a8760bd522925041a52"
BAUD = 512000
HEADER = struct.Struct("<BBBBBIBH")
MAX_PAYLOAD = 2048 - HEADER.size
CONFIG_SIZE, STATE_SIZE, SYNC_SIZE = 228, 87, 315


class Opcode(IntEnum):
    PING = 0x00
    ACK = 0x01
    SYNC = 0x02
    CONFIG = 0x03
    STATE = 0x04
    CLEARERROR = 0x05
    WIFI = 0x06
    ESTOP = 0x20
    STOP = 0x21
    RUN = 0x22
    STEPCLOCK = 0x23
    MOVE = 0x24
    GOTO = 0x25
    GOUNTIL = 0x26
    RELEASESW = 0x27
    GOHOME = 0x28
    GOMARK = 0x29
    RESETPOS = 0x2A
    SETPOS = 0x2B
    SETMARK = 0x2C
    SETCONFIG = 0x2D
    WAITBUSY = 0x2E
    WAITRUNNING = 0x2F
    WAITMS = 0x30
    WAITSWITCH = 0x31
    RUNQUEUE = 0x32
    SETSIGNAL = 0x33
    INCSIGNAL = 0x34
    EMPTYQUEUE = 0x40
    COPYQUEUE = 0x41
    SAVEQUEUE = 0x42
    LOADQUEUE = 0x43


class Direction(IntEnum):
    REVERSE = 0
    FORWARD = 1


class ProtocolError(Exception):
    pass


class ControllerFault(ProtocolError):
    pass


class LinkChanged(ProtocolError):
    pass


def integer(value, low, high, name):
    if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
        raise ValueError(f"{name} must be an integer in [{low}, {high}]")
    return value


def boolean(value):
    if not isinstance(value, bool):
        raise ValueError("expected a bool")
    return int(value)


def encode_direction(value):
    if isinstance(value, bool):
        raise ValueError("use Direction, not a bool")
    return int(Direction(value))


def speed(value):
    value = float(value)
    if not math.isfinite(value) or not 0 <= value <= 15625:
        raise ValueError("speed must be finite and in [0, 15625] full steps/second")
    return value


@dataclass(frozen=True)
class Frame:
    target: int
    queue: int
    command_id: int
    opcode: int
    payload: bytes = b""

    def encode(self):
        integer(self.target, 0, 255, "target")
        integer(self.queue, 0, 255, "queue")
        integer(self.command_id, 0, 0xFFFFFFFF, "command_id")
        integer(self.opcode, 0, 255, "opcode")
        if not isinstance(self.payload, bytes) or len(self.payload) > MAX_PAYLOAD:
            raise ValueError("payload must be bytes and fit the firmware UART buffer")
        tail = struct.pack("<BBBIBH", sum(self.payload) & 255, self.target,
                           self.queue, self.command_id, self.opcode, len(self.payload))
        return bytes((0xAB, sum(tail) & 255)) + tail + self.payload

    @property
    def signed_target(self):
        return self.target if self.target < 128 else self.target - 256


class Parser:
    """Incremental parser. Expire stalled candidates using host monotonic time."""
    def __init__(self, frame_timeout=0.5):
        self.buffer = bytearray()
        self.frame_timeout = frame_timeout
        self.started = None
        self.bad_headers = self.bad_bodies = self.expired = self.discarded = 0

    def _drop(self, count):
        del self.buffer[:count]
        self.started = None
        self.discarded += count

    def feed(self, data=b"", now=None):
        now = time.monotonic() if now is None else now
        self.buffer.extend(data)
        frames = []
        while self.buffer:
            index = self.buffer.find(0xAB)
            if index < 0:
                self._drop(len(self.buffer))
                break
            if index:
                self._drop(index)
            if self.started is None:
                self.started = now
            if now - self.started > self.frame_timeout:
                self.expired += 1
                self._drop(1)
                continue
            if len(self.buffer) < HEADER.size:
                break
            _, check, bodycheck, target, queue, cid, opcode, length = HEADER.unpack_from(self.buffer)
            if (sum(self.buffer[2:12]) & 255) != check or length > MAX_PAYLOAD:
                self.bad_headers += 1
                self._drop(1)
                continue
            total = HEADER.size + length
            if len(self.buffer) < total:
                break
            payload = bytes(self.buffer[12:total])
            if (sum(payload) & 255) != bodycheck:
                self.bad_bodies += 1
                self._drop(1)
                continue
            frames.append(Frame(target, queue, cid, opcode, payload))
            del self.buffer[:total]
            self.started = None
        return frames


@dataclass(frozen=True)
class Identity:
    product: str
    model: str
    branch: str
    version: int
    raw_config: bytes

    @property
    def supported(self):
        return (self.product, self.model, self.branch, self.version) == (
            "Wi-Fi Stepper", "wsx100", "stable", 2)


def decode_config(payload):
    if len(payload) < 110:
        raise ProtocolError("configuration shorter than identity prefix")
    def text(offset):
        return payload[offset:offset + 36].split(b"\0", 1)[0].decode("ascii", "strict")
    return Identity(text(0), text(36), text(72),
                    struct.unpack_from("<H", payload, 108)[0], payload)


@dataclass(frozen=True)
class State:
    errored: bool
    error_when: int
    error_subsystem: int
    error_command_id: int
    error_type: int
    error_arg: int
    this_command: int
    last_command: int
    last_completed: int
    wifi_mode: int
    ip_raw: bytes
    mac: bytes
    chip_id: int
    rssi: int
    direction: int
    movement: int
    hiz: bool
    busy: bool
    user_switch: bool
    step_clock: bool
    alarms: tuple
    steps_per_second: float
    position: int
    mark: int
    vin: float
    signal: int
    signal_modified: int

    @property
    def stopped(self):
        return self.movement == 0


def decode_state(payload):
    """Predicted ESP8266 ABI: 32-bit int/long/ordinary enums; packed status."""
    if len(payload) != STATE_SIZE:
        raise ProtocolError(f"expected {STATE_SIZE} state bytes, got {len(payload)}")
    error = struct.unpack_from("<BIBIib", payload, 0)
    command = struct.unpack_from("<III", payload, 15)
    mode = struct.unpack_from("<I", payload, 27)[0]
    chip, rssi = struct.unpack_from("<Ii", payload, 41)
    flags = struct.unpack_from("<14B", payload, 49)
    if flags[0] > 1 or flags[1] > 3 or any(v > 1 for v in flags[2:]):
        raise ProtocolError("invalid status fields: possible incompatible ABI")
    sps, pos, mark, vin = struct.unpack_from("<fiif", payload, 63)
    signal_value, modified = struct.unpack_from("<iI", payload, 79)
    if error[0] > 1 or not all(math.isfinite(v) for v in (sps, vin)):
        raise ProtocolError("invalid state values")
    return State(bool(error[0]), *error[1:], *command, mode,
                 payload[31:35], payload[35:41], chip, rssi,
                 flags[0], flags[1], *map(bool, flags[2:6]),
                 tuple(map(bool, flags[6:])), sps, pos, mark, vin,
                 signal_value, modified)


class Host:
    """Synchronous, single-thread-owner UART master.

    A serial-like object must implement read(), write(), close(), in_waiting,
    and have bounded read/write timeouts. Use open_serial() for pyserial.
    No movement/configuration is sent on construction or close.
    """
    def __init__(self, serial_port, *, allow_control=False, trace=None):
        self.serial = serial_port
        self.allow_control = allow_control
        self.trace = trace
        self.parser = Parser()
        self.slaves = None
        self.epoch = 0
        self.identities = {}
        self.states = {}  # target -> (host monotonic receive time, State)
        self.diagnostics = deque(maxlen=100)
        self._heartbeat = False
        self._last_tx = self._last_ping_rx = float("-inf")
        self._candidate = None
        self._candidate_count = 0
        self._next_id = 0
        self._waiting = None
        self._reply = None

    @classmethod
    def open_serial(cls, port, **kwargs):
        import serial
        dev = serial.Serial(port, baudrate=BAUD, bytesize=8, parity="N", stopbits=1,
                            timeout=0.01, write_timeout=0.25, xonxoff=False,
                            rtscts=False, dsrdtr=False, exclusive=True)
        return cls(dev, **kwargs)

    def _write(self, frame):
        raw = frame.encode()
        if self.trace:
            self.trace("tx", time.monotonic(), raw)
        # A short write leaves a partial frame on the wire: fail, do not retry.
        if self.serial.write(raw) != len(raw):
            raise OSError("short UART write; connection state uncertain")

    def _receive(self, frame, now):
        if frame.opcode == Opcode.PING and frame.command_id == 0 and not frame.payload:
            count = -frame.signed_target
            if not 1 <= count <= 32:
                self.diagnostics.append("ignored ping with invalid/empty ring")
                return
            self._last_ping_rx = now
            if count == self._candidate:
                self._candidate_count += 1
            else:
                self._candidate, self._candidate_count = count, 1
            if self.slaves is not None and count != self.slaves:
                self.slaves = None
                self.identities.clear()
                self.states.clear()
                self.epoch += 1
            if self._candidate_count >= 2 and self.slaves is None:
                self.slaves = count
            return
        if self.slaves is None:
            return
        target = self.slaves + frame.signed_target
        if frame.signed_target > 0 or not 1 <= target <= self.slaves:
            self.diagnostics.append("ignored invalid return route")
            return
        if self._waiting == (target, frame.command_id, frame.opcode):
            self._reply = frame
        try:
            if frame.opcode == Opcode.CONFIG:
                identity = decode_config(frame.payload)
                if len(frame.payload) != CONFIG_SIZE or not identity.supported:
                    self.identities.pop(target, None)
                    raise ProtocolError("configuration does not match stable-002 profile")
                previous = self.identities.get(target)
                if previous is not None and previous != identity:
                    self.states.pop(target, None)
                self.identities[target] = identity
            elif frame.opcode == Opcode.STATE:
                self.states[target] = (now, decode_state(frame.payload))
        except (ProtocolError, UnicodeError) as error:
            self.diagnostics.append(str(error))
            self.identities.pop(target, None)
            self.states.pop(target, None)

    def poll(self):
        now = time.monotonic()
        if self._heartbeat and now - self._last_tx >= 0.25:
            self._write(Frame(0, 0, 0, Opcode.PING))
            self._last_tx = now
        if self.slaves is not None and now - self._last_ping_rx > 1.0:
            self.slaves = None
            self.identities.clear()
            self.states.clear()
            self._candidate, self._candidate_count = None, 0
            self.epoch += 1
        data = self.serial.read(min(max(self.serial.in_waiting, 1), 4096))
        now = time.monotonic()
        if data and self.trace:
            self.trace("rx", now, bytes(data))
        for frame in self.parser.feed(data, now):
            self._receive(frame, now)

    def sleep(self, seconds):
        """Wait while continuing RX and heartbeats; unlike time.sleep()."""
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            self.poll()
            time.sleep(0.001)

    def discover(self, timeout=3.0):
        self._heartbeat = True
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.poll()
            if self.slaves is not None:
                return self.slaves
            time.sleep(0.001)
        raise TimeoutError("no consistent returned discovery ping")

    def _target(self, target):
        integer(target, 1, 32, "target")
        if self.slaves is None or target > self.slaves:
            raise LinkChanged("discover the ring first")
        if time.monotonic() - self._last_ping_rx > 1.0:
            raise LinkChanged("returned heartbeat is stale")

    def _request(self, target, opcode, payload=b"", queue=0, reply_opcode=Opcode.ACK,
                 timeout=2.0):
        self._target(target)
        integer(queue, 0, 15, "queue")
        if self._waiting is not None:
            raise RuntimeError("only one request at a time")
        self._next_id = self._next_id % 0xFFFFFFFF + 1
        cid, epoch = self._next_id, self.epoch
        self._reply = None
        self._waiting = (target, cid, int(reply_opcode))
        try:
            self._write(Frame(target, queue, cid, int(opcode), payload))
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                self.poll()
                if self.epoch != epoch:
                    raise LinkChanged("ring changed during request; outcome unknown")
                if self._reply is not None:
                    result = self._reply
                    if reply_opcode == Opcode.ACK and result.payload:
                        raise ProtocolError("ACK must have no payload")
                    return result
                time.sleep(0.001)
            raise TimeoutError(f"no reply for command {cid}; outcome unknown; not retried")
        finally:
            self._waiting = self._reply = None

    def identify(self, target=1):
        reply = self._request(target, Opcode.SYNC, reply_opcode=Opcode.SYNC)
        identity = decode_config(reply.payload)
        if len(reply.payload) != SYNC_SIZE or not identity.supported:
            self.identities.pop(target, None)
            raise ProtocolError(f"unsupported identity/layout: {identity}")
        self.identities[target] = decode_config(reply.payload[:CONFIG_SIZE])
        # SYNC leaves software signal zeroed in firmware. Wait for real STATE.
        return self.identities[target]

    def state(self, target=1, max_age=1.0):
        self._target(target)
        if target not in self.identities:
            raise ProtocolError("identify stable-002 before reading decoded state")
        entry = self.states.get(target)
        if entry is None or time.monotonic() - entry[0] > max_age:
            raise ProtocolError("no fresh periodic state report")
        return entry[1]

    def _control(self, target, opcode, payload=b"", queue=0):
        if not self.allow_control:
            raise PermissionError("open Host with allow_control=True to issue operations")
        self._target(target)
        if target not in self.identities:
            raise ProtocolError("identify stable-002 first")
        return self._request(target, opcode, payload, queue).command_id

    def move(self, microsteps, *, direction=Direction.FORWARD, target=1, queue=0):
        integer(microsteps, 0, 0x3FFFFF, "microsteps")
        return self._control(target, Opcode.MOVE,
                             struct.pack("<BI", encode_direction(direction), microsteps), queue)

    def run(self, steps_per_second, *, direction=Direction.FORWARD, target=1, queue=0):
        return self._control(target, Opcode.RUN,
                             struct.pack("<Bf", encode_direction(direction), speed(steps_per_second)), queue)

    def step_clock(self, *, direction=Direction.FORWARD, target=1, queue=0):
        """Enter external step-clock mode; requires separate hardware pulses."""
        return self._control(target, Opcode.STEPCLOCK, bytes((encode_direction(direction),)), queue)

    def goto(self, position, *, direction=None, target=1, queue=0):
        integer(position, -0x200000, 0x1FFFFF, "position")
        d = Direction.FORWARD if direction is None else direction
        return self._control(target, Opcode.GOTO,
                             struct.pack("<BBi", int(direction is not None), encode_direction(d), position), queue)

    def stop(self, *, hiz=False, soft=True, target=1, queue=0):
        """Queued stop. hiz=False retains energized holding, subject to configuration."""
        return self._control(target, Opcode.STOP, bytes((boolean(hiz), boolean(soft))), queue)

    def immediate_stop(self, *, hiz=False, soft=False, target=1):
        """Firmware ESTOP: bypass queue then empty Q0. NOT a safety-rated e-stop."""
        return self._control(target, Opcode.ESTOP, bytes((boolean(hiz), boolean(soft))))

    def wait_not_busy(self, *, target=1, queue=0):
        """Enqueue WAITBUSY; this Python call waits for ACK, not motion completion."""
        return self._control(target, Opcode.WAITBUSY, queue=queue)

    def wait_stopped(self, *, target=1, queue=0):
        """Enqueue WAITRUNNING: despite its legacy name it waits for STOPPED."""
        return self._control(target, Opcode.WAITRUNNING, queue=queue)

    def wait_ms(self, milliseconds, *, target=1, queue=0):
        return self._control(target, Opcode.WAITMS,
                             struct.pack("<I", integer(milliseconds, 0, 0xFFFFFFFF, "milliseconds")), queue)

    def set_signal(self, value, *, target=1, queue=0):
        return self._control(target, Opcode.SETSIGNAL,
                             struct.pack("<i", integer(value, -0x80000000, 0x7FFFFFFF, "value")), queue)

    def increment_signal(self, value, *, target=1, queue=0):
        return self._control(target, Opcode.INCSIGNAL,
                             struct.pack("<i", integer(value, -0x80000000, 0x7FFFFFFF, "value")), queue)

    def set_position(self, position, *, target=1, queue=0):
        return self._control(target, Opcode.SETPOS,
                             struct.pack("<i", integer(position, -0x200000, 0x1FFFFF, "position")), queue)

    def set_mark(self, position, *, target=1, queue=0):
        return self._control(target, Opcode.SETMARK,
                             struct.pack("<i", integer(position, -0x200000, 0x1FFFFF, "position")), queue)

    def go_home(self, *, target=1, queue=0):
        """Go to numeric zero; NOT a limit-switch homing procedure."""
        return self._control(target, Opcode.GOHOME, queue=queue)

    def go_mark(self, *, target=1, queue=0):
        return self._control(target, Opcode.GOMARK, queue=queue)

    def reset_position(self, *, target=1, queue=0):
        return self._control(target, Opcode.RESETPOS, queue=queue)

    def wait_switch(self, closed, *, target=1, queue=0):
        return self._control(target, Opcode.WAITSWITCH, bytes((boolean(closed),)), queue)

    def run_queue(self, stored_queue, *, target=1, queue=0):
        return self._control(target, Opcode.RUNQUEUE,
                             bytes((integer(stored_queue, 1, 15, "stored_queue"),)), queue)

    def empty_queue(self, *, target=1, queue=0):
        """Empty commands, not a motor stop."""
        return self._control(target, Opcode.EMPTYQUEUE, queue=queue)

    def clear_error(self, *, target=1):
        return self._control(target, Opcode.CLEARERROR)

    def wait_signal(self, value, *, target=1, timeout=10.0):
        """Observe a unique marker after a queue barrier; not an exactly-once guarantee."""
        self._target(target)
        epoch = self.epoch
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.poll()
            if self.epoch != epoch:
                raise LinkChanged("ring changed while waiting for signal")
            entry = self.states.get(target)
            if entry is None or time.monotonic() - entry[0] > 1.0:
                time.sleep(0.001)
                continue
            state = self.state(target)
            if state.errored or any(state.alarms[i] for i in (0, 1, 2, 3)):
                raise ControllerFault(str(state))
            if state.signal == value:
                return state
            time.sleep(0.001)
        raise TimeoutError("signal not observed; controller may still be executing")

    def close(self):
        self._heartbeat = False
        self.serial.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
