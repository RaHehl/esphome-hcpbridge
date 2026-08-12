#!/usr/bin/env python3
"""Compare the two sides against an expectation, not against equality.

Equality stopped being the right assertion the moment the responder was allowed
to stay silent on purpose. A plain diff then reports thousands of frames and
says nothing, and the one frame worth looking at drowns in it.

So each difference is classified. A difference is expected when the new side is
silent for a reason this file can name, or when both answered and the change is
one of the deliberate ones listed below. Anything else fails the run.

Register and state columns are reported but never fail: once the old side
accepts a write the new side refuses, the two register maps never realign, so
every later line differs for a reason that has nothing to do with that line.

argv: <frames file> <old output> <new output>
"""
import sys

READ_BLOCK, WRITE_BLOCK, BCAST_BLOCK = 0x9CB9, 0x9C41, 0x9D31
MAX_REGS = 9
MAX_READ = 8


def be16(b, i):
    return (b[i] << 8) | b[i + 1]


def why_silence(f):
    """Name the reason the new side may stay silent, or None if it may not."""
    if len(f) < 4:
        return "frame shorter than an address, a function code and a checksum"
    fc = f[1]
    if fc not in (0x17, 0x10):
        return "function code %02x is not on this bus" % fc
    body = f[:-2]  # strip the checksum
    if fc == 0x17:
        if len(body) < 11:
            return "read/write frame too short for its header"
        read_addr, read_cnt = be16(body, 2), be16(body, 4)
        write_addr, write_cnt = be16(body, 6), be16(body, 8)
        byte_cnt = body[10]
        if read_addr != READ_BLOCK or write_addr != WRITE_BLOCK:
            return "names register block %04x/%04x" % (read_addr, write_addr)
        if read_cnt < 1 or read_cnt > MAX_READ or write_cnt < 1 or byte_cnt != 2 * write_cnt:
            return "counts do not agree with each other"
        if byte_cnt < 2 or byte_cnt > 2 * MAX_REGS:
            return "byte count %d outside 2..%d" % (byte_cnt, 2 * MAX_REGS)
        if len(body) < 11 + byte_cnt:
            return "frame shorter than the byte count claims"
        return None
    if len(body) < 7:
        return "broadcast too short for its header"
    addr, cnt, byte_cnt = be16(body, 2), be16(body, 4), body[6]
    if addr != BCAST_BLOCK:
        return "names register block %04x" % addr
    if cnt < 1 or byte_cnt != 2 * cnt or byte_cnt < 2 or byte_cnt > 2 * MAX_REGS:
        return "counts do not agree with each other"
    if len(body) < 7 + byte_cnt:
        return "frame shorter than the byte count claims"
    return None


def deliberate(frame, old_r, new_r):
    """Both sides answered and the bytes differ. Is that one of ours?"""
    # The identity request rides along in an ordinary status answer: answer code
    # 0x22 in the second register, the request code in the third. Read at the
    # register the code actually sits in, not four characters further on.
    if len(new_r) >= 18 and new_r[10:14] == "0322" and new_r[14:16] == "05":
        return "identity request riding in the answer"
    body = frame[:-2]
    if len(body) >= 13 and body[12] not in (0x02, 0x03, 0x04):
        # Nothing knows what to answer, so the block is cleared and the old
        # side's leftovers are not comparable. Only excused while the block
        # really is clear: a door command in there is a defect, not a
        # difference, and this used to wave through 296 of them.
        if len(new_r) >= 22 and new_r[14:22] == "00000000":
            return "command %02x is not one we answer, block cleared" % body[12]
    # The counter's top bit selects the half of a split payload and is not part
    # of the count, so it is no longer echoed. Everything else has to match.
    if len(old_r) >= 12 and len(new_r) == len(old_r):
        o_body, n_body = old_r[:-4], new_r[:-4]   # without the checksum
        if (o_body[:6] == n_body[:6] and o_body[8:] == n_body[8:]
                and int(o_body[6:8], 16) ^ int(n_body[6:8], 16) == 0x80):
            return "counter top bit no longer echoed"
    if len(old_r) >= 22 and len(new_r) >= 22:
        old2, new2 = old_r[14:18], new_r[14:18]
        if old2.startswith("02") and new2.startswith("01"):
            # The key press used to go out as two frames, a press and a
            # release. It is one frame now, carrying what used to be the second.
            return "two-phase key press replaced by a single frame"
        if old2 == "0800" and new2 in ("0880", "0800"):
            return "light toggle replaced by separate on and off"
    return None


def main():
    frames_path, old_path, new_path = sys.argv[1], sys.argv[2], sys.argv[3]
    frames = [l.strip() for l in open(frames_path) if l.strip() and l[0] not in "TC"]
    old = [l.rstrip("\n") for l in open(old_path)]
    new = [l.rstrip("\n") for l in open(new_path)]
    if not (len(frames) == len(old) == len(new)):
        print("  MISMATCHED LINE COUNTS: %d frames, %d old, %d new"
              % (len(frames), len(old), len(new)))
        return 1

    expected, drift, unexpected = {}, 0, []
    for hexf, o, n in zip(frames, old, new):
        o_r = o.split()[0][2:]
        n_r = n.split()[0][2:]
        if o_r == n_r:
            if o != n:
                drift += 1
            continue
        if n_r == "-":
            reason = why_silence(bytes.fromhex(hexf))
            if reason is None:
                unexpected.append((hexf, o_r, n_r, "new side silent with no reason to be"))
            else:
                expected[reason] = expected.get(reason, 0) + 1
            continue
        if o_r == "-":
            unexpected.append((hexf, o_r, n_r, "new side answers where the old one did not"))
            continue
        reason = deliberate(bytes.fromhex(hexf), o_r, n_r)
        if reason is None:
            unexpected.append((hexf, o_r, n_r, "both answered, bytes differ"))
        else:
            expected[reason] = expected.get(reason, 0) + 1

    total_expected = sum(expected.values())
    print("  %d frames: %d expected difference(s), %d unexpected, %d register drift"
          % (len(frames), total_expected, len(unexpected), drift))
    for reason, count in sorted(expected.items(), key=lambda kv: -kv[1]):
        print("    %6d  %s" % (count, reason))
    for hexf, o_r, n_r, what in unexpected[:10]:
        print("    UNEXPECTED %s" % what)
        print("      frame %s" % hexf)
        print("      old   %s" % o_r)
        print("      new   %s" % n_r)
    return 1 if unexpected else 0


if __name__ == "__main__":
    sys.exit(main())
