# UART wire notes

Reference source: commit 3627d32739669bb249ec7a8760bd522925041a52.

Header `<BBBBBIBH` is 12 bytes: magic, head sum, body sum, target, queue,
32-bit command ID, opcode, 16-bit payload length. Little endian.
Body sum = sum(payload) modulo 256. Head sum = sum(header[2:12]) modulo 256.
Header magic AB and checksum field itself are excluded from the header sum.

Canonical vectors:
- Discovery TX: `ab 00 00 00 00 00 00 00 00 00 00 00`
- One-slave discovery return: `ab ff 00 ff 00 00 00 00 00 00 00 00`
- Target-1 SYNC, ID 1: `ab 04 00 01 00 01 00 00 00 02 00 00`
- One-slave ACK, ID 1: `ab 02 00 00 00 01 00 00 00 01 00 00`

Master PING: target=0, queue=0, ID=0, opcode=0, no body. Each slave marks the
link active and decrements a forwarded target; firmware master uses signed
negative target to discover slave count. Normal requests target 1..N.
Slave-originated frames start target=0; downstream slaves decrement it.
Origin index = N + signed(return target). For N=1 all slave-originated frames
arrive target=0, while discovery returns target=-1. For N=3 origin 1 returns
-2, origin 2 returns -1, origin 3 returns 0. No broadcast motor command assumed.

Nominal firmware intervals: ping >250 ms; inactive after >1000 ms; STATE >250
ms; CONFIG >2000 ms. Cooperative-loop delays can increase those times. Host
uses 250-ms ping period, 1-s stale threshold, 2-s request deadline. Host frame
candidate expiry is 0.5 s; it is a parser recovery choice, not wire framing.

SYNC request empty; reply opcode SYNC with same command ID. ACK is separate
opcode 01 with empty body and original command ID. Periodic CONFIG/STATE use
ID zero. Do not ACK these reports. Unknown packets are not automatically echoed
or forwarded by the host. A Pi ring host terminates returned packets.

Predicted stable-002 ABI (must be checked on installed firmware):
- 32-bit ordinary enums, int, long; 1-byte packed enums and bool; float32.
- Nested cm/vm config structures preserve their own natural alignment.
- config 228 bytes, state 87 bytes, combined SYNC 315 bytes.
- Identity prefix 36s,36s,36s,uint16 at offsets 0,36,72,108.
- State blocks: error 0:15; command 15:27; wifi 27:49; motor 49:79;
  signal 79:87.
- Motor block status: direction, movement, hiz, busy, switch, step-clock,
  then eight Boolean alarms: command_error, overcurrent, undervoltage,
  thermal_shutdown, user_switch, thermal_warning, stall_detect, adc_undervoltage.
- Motor float speed at 63; int position 67; int mark 71; float Vin 75.
- IP is retained as raw bytes, not converted with an unverified endian rule.
- SYNC signal state is zeroed rather than copied; use periodic STATE.

Operation preconditions come from command.h, not the wire opcode bits.
Wire MOVE is 24; internal queued MOVE also has precondition bits. They are
NOT interchangeable encodings. WAITMS UART body is uint32 ms, not its internal
queue representation with a started timestamp. Host IDs are uint32 nonzero
except PING; wrap skips zero, with only one outstanding request.

ACK is handler receipt, not guaranteed queue insertion. No duplicate
suppression or retransmission guarantee is assumed. No motor stop on lost
heartbeat is present in the inspected daisy timeout path. Queue execution and
physical motor movement are distinct. See README for queued vs immediate stop.
