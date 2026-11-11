import struct
import unittest
from unittest.mock import patch
import wifistepper_uart as w


def configuration():
    raw = bytearray(w.CONFIG_SIZE)
    for offset, text in [(0, b"Wi-Fi Stepper"), (36, b"wsx100"), (72, b"stable")]:
        raw[offset:offset + len(text)] = text
    struct.pack_into("<H", raw, 108, 2)
    return bytes(raw)


def state_payload(signal=0):
    raw = bytearray(w.STATE_SIZE)
    struct.pack_into("<BIBIib", raw, 0, 0, 1234, 0, 0, 0, -1)
    struct.pack_into("<III", raw, 15, 0, 55, 1234)
    struct.pack_into("<I", raw, 27, 2)
    raw[31:35] = bytes([192, 168, 1, 20])
    raw[35:41] = bytes(range(6))
    struct.pack_into("<Ii", raw, 41, 0x123456, -45)
    struct.pack_into("<14B", raw, 49, 1, 0, 0, 0, 0, 0, *([0] * 8))
    struct.pack_into("<fiif", raw, 63, 0.0, -128, 256, 24.0)
    struct.pack_into("<iI", raw, 79, signal, 1234)
    return bytes(raw)


class Clock:
    def __init__(self): self.now = 0.0
    def monotonic(self): return self.now
    def sleep(self, seconds): self.now += seconds


class FakeSerial:
    """One synthetic slave. Not a firmware emulator or hardware test."""
    def __init__(self, clock):
        self.clock, self.rx, self.writes = clock, bytearray(), []
        self.signal = 0
        self.closed = False
        self.reply_commands = True
    @property
    def in_waiting(self): return len(self.rx)
    def read(self, n):
        self.clock.now += 0.01
        out = bytes(self.rx[:n]); del self.rx[:n]
        return out
    def enqueue(self, frame): self.rx.extend(frame.encode())
    def write(self, data):
        frame = w.Parser().feed(data, now=0)[0]
        self.writes.append(frame)
        if frame.opcode == w.Opcode.PING:
            self.enqueue(w.Frame(255, 0, 0, w.Opcode.PING))
            self.enqueue(w.Frame(0, 0, 0, w.Opcode.STATE, state_payload(self.signal)))
        elif frame.opcode == w.Opcode.SYNC:
            self.enqueue(w.Frame(0, 0, frame.command_id, w.Opcode.SYNC,
                                 configuration() + state_payload()))
        elif self.reply_commands:
            if frame.opcode == w.Opcode.SETSIGNAL:
                self.signal = struct.unpack("<i", frame.payload)[0]
            self.enqueue(w.Frame(0, frame.queue, frame.command_id, w.Opcode.ACK))
        return len(data)
    def close(self): self.closed = True


class CodecTests(unittest.TestCase):
    def test_ping_vector(self):
        self.assertEqual(w.Frame(0, 0, 0, 0).encode().hex(), "ab0000000000000000000000")
    def test_sync_vector(self):
        self.assertEqual(w.Frame(1, 0, 1, 2).encode().hex(), "ab0400010001000000020000")
    def test_return_vector(self):
        self.assertEqual(w.Frame(255, 0, 0, 0).encode().hex(), "abff00ff0000000000000000")
    def test_all_fragment_boundaries(self):
        f = w.Frame(1, 3, 0x12345678, w.Opcode.MOVE, b"\x01\xab\0\0\0")
        raw = f.encode()
        for i in range(len(raw)+1):
            p = w.Parser()
            self.assertEqual(p.feed(raw[:i], 0) + p.feed(raw[i:], .1), [f])
    def test_bytewise(self):
        f = w.Frame(1, 0, 1, w.Opcode.RUN, struct.pack('<Bf', 1, 100))
        p = w.Parser(); got = []
        for b in f.encode(): got += p.feed(bytes([b]), .01)
        self.assertEqual(got, [f])
    def test_noise_and_multiple(self):
        f = w.Frame(0, 0, 0, 0)
        p = w.Parser()
        self.assertEqual(p.feed(b"boot\r\n"+f.encode()*2, 0), [f,f])
        self.assertEqual(p.discarded, 6)
    def test_corrupt_header(self):
        f = w.Frame(0, 0, 0, 0); bad=bytearray(f.encode()); bad[1] ^= 1
        p=w.Parser(); self.assertEqual(p.feed(bad+f.encode(), 0), [f])
        self.assertEqual(p.bad_headers, 1)
    def test_corrupt_body(self):
        f=w.Frame(1, 0, 1, 0x24, b'abc'); bad=bytearray(f.encode()); bad[-1]^=1
        p=w.Parser(); self.assertEqual(p.feed(bad+f.encode(), 0), [f])
        self.assertEqual(p.bad_bodies, 1)
    def test_stalled_candidate(self):
        p=w.Parser(); p.feed(b'\xab', 0)
        f=w.Frame(0, 0, 0, 0)
        self.assertEqual(p.feed(f.encode(), 1), [f])
        self.assertEqual(p.expired, 1)
    def test_oversized(self):
        with self.assertRaises(ValueError): w.Frame(1,0,1,2,b'x'*2037).encode()
    def test_identity(self):
        self.assertTrue(w.decode_config(configuration()).supported)
    def test_state(self):
        s=w.decode_state(state_payload(-42))
        self.assertEqual((s.position,s.mark,s.signal,s.chip_id,s.rssi),(-128,256,-42,0x123456,-45))
        self.assertTrue(s.stopped); self.assertFalse(s.busy)
    def test_bad_state_size(self):
        with self.assertRaises(w.ProtocolError): w.decode_state(b'')
    def test_bad_state_flag(self):
        raw=bytearray(state_payload()); raw[52]=2
        with self.assertRaises(w.ProtocolError): w.decode_state(raw)


class HostTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock()
        self.patches=[patch.object(w.time,'monotonic',self.clock.monotonic),
                      patch.object(w.time,'sleep',self.clock.sleep)]
        for p in self.patches: p.start()
        self.dev=FakeSerial(self.clock)
        self.host=w.Host(self.dev)
    def tearDown(self):
        for p in reversed(self.patches): p.stop()
    def ready(self, control=True):
        self.assertEqual(self.host.discover(),1)
        self.assertTrue(self.host.identify().supported)
        self.host.allow_control=control
    def test_discovery_identification_state(self):
        self.ready(False)
        self.assertEqual(self.host.state().position,-128)
    def test_control_disabled(self):
        self.ready(False)
        with self.assertRaises(PermissionError): self.host.move(1)
    def test_move_payload_and_ack_id(self):
        self.ready(); cid=self.host.move(128,direction=w.Direction.REVERSE)
        f=self.dev.writes[-1]
        self.assertEqual((f.opcode,f.payload,f.command_id),(0x24,b'\0\x80\0\0\0',cid))
    def test_goto_auto(self):
        self.ready(); self.host.goto(-128)
        self.assertEqual(self.dev.writes[-1].payload,struct.pack('<BBi',0,1,-128))
    def test_run_units(self):
        self.ready(); self.host.run(100)
        self.assertEqual(self.dev.writes[-1].payload,struct.pack('<Bf',1,100))
    def test_stop_distinction(self):
        self.ready(); self.host.stop(); f=self.dev.writes[-1]
        self.assertEqual((f.opcode,f.payload),(0x21,b'\0\x01'))
        self.host.immediate_stop(); f=self.dev.writes[-1]
        self.assertEqual((f.opcode,f.payload),(0x20,b'\0\0'))
    def test_wait_opcodes(self):
        self.ready(); self.host.wait_not_busy(); self.assertEqual(self.dev.writes[-1].opcode,0x2e)
        self.host.wait_stopped(); self.assertEqual(self.dev.writes[-1].opcode,0x2f)
    def test_no_retry_timeout(self):
        self.ready(); self.dev.reply_commands=False
        with self.assertRaises(TimeoutError): self.host.move(1)
        self.assertEqual(sum(f.opcode==w.Opcode.MOVE for f in self.dev.writes),1)
    def test_bounds(self):
        self.ready()
        for value in [-1,0x400000,1.2,True]:
            with self.assertRaises(ValueError): self.host.move(value)
        for value in [float('nan'),float('inf'),-1]:
            with self.assertRaises(ValueError): self.host.run(value)
    def test_close_no_stop(self):
        self.ready(); n=len(self.dev.writes); self.host.close()
        self.assertEqual(len(self.dev.writes),n); self.assertTrue(self.dev.closed)
    def test_wait_signal_with_heartbeats(self):
        self.ready(); self.host.set_signal(12345)
        self.assertEqual(self.host.wait_signal(12345).signal,12345)
    def test_topology_change_invalidates(self):
        self.ready(); self.host._receive(w.Frame(254,0,0,w.Opcode.PING),self.clock.now)
        self.assertIsNone(self.host.slaves); self.assertFalse(self.host.identities)
    def test_stale_heartbeat(self):
        self.ready(); self.clock.now+=2
        with self.assertRaises(w.LinkChanged): self.host.move(1)
    def test_short_write_fails_without_retry(self):
        self.ready()
        self.dev.write=lambda data: len(data)-1
        with self.assertRaises(OSError): self.host.move(1)
    def test_invalid_queue(self):
        self.ready()
        with self.assertRaises(ValueError): self.host.move(1,queue=16)
    def test_wrong_state_layout_revokes_identity(self):
        self.ready()
        self.host._receive(w.Frame(0,0,0,w.Opcode.STATE,b'x'*84),self.clock.now)
        self.assertFalse(self.host.identities)
    def test_no_commands_on_construction(self):
        self.assertEqual(self.dev.writes,[])
    def test_id_wrap(self):
        self.ready(); self.host._next_id=0xffffffff
        self.assertEqual(self.host.move(0),1)
    def test_wrong_identity_disables(self):
        self.ready(); raw=bytearray(configuration()); raw[108]=6
        self.host._receive(w.Frame(0,0,0,w.Opcode.CONFIG,bytes(raw)),self.clock.now)
        with self.assertRaises(w.ProtocolError): self.host.move(1)

if __name__=='__main__': unittest.main()
