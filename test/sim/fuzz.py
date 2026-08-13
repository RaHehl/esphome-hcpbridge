"""Frames no drive would send, and what the accessory is allowed to do with them.

The drive model only produces well-formed traffic, so on its own it says
nothing about a corrupted frame, a truncated one, or another device's answer
picked up from a shared bus. Those are the inputs that once let the old
implementation read past its own buffer.

The oracle here is deliberately not "whatever the code does". A frame may go
unanswered only for a reason this file can name from the frame itself; an
answer to something unanswerable, or silence where an answer was owed, fails.
Built with sanitizers, the same corpus also has to come back without a memory
error, which is the other half of what these inputs are for.
"""

import random
import sys

import drive
import hcp

READ_BLOCK, WRITE_BLOCK, BCAST_BLOCK = hcp.REG_RESP, hcp.REG_CMD, hcp.REG_BCAST
MAX_REGS = 9
MAX_READ = 8


def be16(b, i):
    return (b[i] << 8) | b[i + 1]


def why_silence(f):
    """Name the reason this frame may go unanswered, or None if it may not.

    A positive statement of what the bus carries, written from the protocol and
    not from the responder: two function codes, three register blocks, and
    counts that have to agree with each other.
    """
    if len(f) < 4:
        return "shorter than an address, a function code and a checksum"
    if crc_bad(f):
        return "checksum does not match"
    fc = f[1]
    if f[0] not in (hcp.SLAVE, 0x00):
        return f"addressed to {f[0]:02x}, another device on the bus"
    if fc not in (0x17, 0x10):
        return f"function code {fc:02x} is not on this bus"
    body = f[:-2]
    if fc == 0x17:
        if len(body) < 11:
            return "read/write frame too short for its header"
        read_addr, read_cnt = be16(body, 2), be16(body, 4)
        write_addr, write_cnt = be16(body, 6), be16(body, 8)
        byte_cnt = body[10]
        if read_addr != READ_BLOCK or write_addr != WRITE_BLOCK:
            return f"names register block {read_addr:04x}/{write_addr:04x}"
        if (
            read_cnt < 1
            or read_cnt > MAX_READ
            or write_cnt < 1
            or byte_cnt != 2 * write_cnt
        ):
            return "counts do not agree with each other"
        if byte_cnt < 2 or byte_cnt > 2 * MAX_REGS:
            return f"byte count {byte_cnt} outside 2..{2 * MAX_REGS}"
        if len(body) < 11 + byte_cnt:
            return "shorter than the byte count claims"
        return None
    if len(body) < 7:
        return "broadcast too short for its header"
    addr, cnt, byte_cnt = be16(body, 2), be16(body, 4), body[6]
    if addr != BCAST_BLOCK:
        return f"names register block {addr:04x}"
    if cnt < 1 or byte_cnt != 2 * cnt or byte_cnt < 2 or byte_cnt > 2 * MAX_REGS:
        return "counts do not agree with each other"
    if len(body) < 7 + byte_cnt:
        return "shorter than the byte count claims"
    # A broadcast goes to everyone and is never answered.
    return "a broadcast is not addressed to anybody"


def crc_bad(f):
    return hcp.crc16(f[:-2]) != (f[-2] | (f[-1] << 8))


def corpus(rng, count):
    """Frames built to land near the edges rather than uniformly at random.

    Purely random bytes almost never produce a frame that gets far enough into
    the parser to be interesting, so most of these start from something valid
    and then break one thing.
    """
    out = []
    for _ in range(count):
        kind = rng.randrange(6)
        if kind == 0:
            out.append(bytes(rng.randrange(256) for _ in range(rng.randrange(1, 40))))
            continue
        base = bytearray(hcp.status_poll(rng.randrange(0x80)))
        if kind == 1:
            base = base[: rng.randrange(1, len(base))]  # cut short
        elif kind == 2:
            base[rng.randrange(len(base))] ^= 1 << rng.randrange(8)  # one bit
        elif kind == 3:
            base[10] = rng.randrange(256)  # byte count lies
        elif kind == 4:
            base[1] = rng.randrange(256)  # other function code
        elif kind == 5:
            base += bytes(rng.randrange(256) for _ in range(rng.randrange(1, 8)))
        out.append(bytes(base))
    return out


def main():
    rounds = int(sys.argv[1]) if len(sys.argv) > 1 else 4000
    binary = sys.argv[2] if len(sys.argv) > 2 else "./accessory"
    rng = random.Random(20260813)
    acc = drive.Accessory(binary)
    # A link that is already up, so a refusal cannot be explained away by the
    # accessory not having heard anything yet.
    d = drive.Drive(acc)
    d.run(4)

    bad = []
    frames = corpus(rng, rounds)
    for f in frames:
        try:
            answer = acc.exchange(f)
        except Exception as e:
            bad.append((f.hex(), f"the accessory died: {e!r}"))
            break
        reason = why_silence(f)
        if answer.silent and reason is None:
            bad.append((f.hex(), "no answer, and the frame gives no reason"))
        elif not answer.silent and reason is not None:
            bad.append((f.hex(), f"answered a frame that {reason}"))
    acc.close()

    print(f"  {len(frames)} malformed and foreign frames")
    for hexf, what in bad[:10]:
        print(f"    UNEXPECTED {what}\n      {hexf}")
    if bad:
        print(f"  {len(bad)} unexpected")
        return 1
    print("  none answered that should not have been, none refused that should")
    return 0


if __name__ == "__main__":
    sys.exit(main())
