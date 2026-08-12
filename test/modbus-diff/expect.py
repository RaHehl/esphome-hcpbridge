#!/usr/bin/env python3
"""Behaviour the differential test cannot reach.

The old implementation has no identity exchange, no pause handshake and no
notion of a lost answer, so there is nothing to compare those against. They are
also the bulk of what changed. This drives the new side alone and asserts what
each answer has to contain.

Every case here is one that was reasoned about and then reproduced, so a
regression shows up as a named failure rather than as a number that moved.
"""
import subprocess
import sys

SLAVE = 0x02
READ, WRITE, BCAST = 0x9CB9, 0x9C41, 0x9D31
# new_main.cpp: C<n> picks from this list.
OPEN, CLOSE, IMPULSE, HALF, VENT, LAMPON = range(6)


def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c


def wrap(body):
    c = crc16(body)
    return (body + bytes([c & 0xFF, c >> 8])).hex()


def poll(counter, command=0x03, read_cnt=8, payload=(0x00, 0x00)):
    body = bytes([SLAVE, 0x17, READ >> 8, READ & 0xFF, 0x00, read_cnt,
                  WRITE >> 8, WRITE & 0xFF, 0x00, 1 + len(payload) // 2,
                  2 + len(payload), counter, command]) + bytes(payload)
    return wrap(body)


def transfer(counter, sub, payload):
    """A data transfer: command 0x04, sub code, then the payload registers."""
    regs = 2 + len(payload) // 2
    body = bytes([SLAVE, 0x17, READ >> 8, READ & 0xFF, 0x00, 8,
                  WRITE >> 8, WRITE & 0xFF, 0x00, regs, 2 * regs,
                  counter, 0x04, sub, 0x00]) + bytes(payload)
    return wrap(body)


def run(lines):
    out = subprocess.run(["./new_bin"], input="\n".join(lines) + "\n",
                         capture_output=True, text=True, check=True).stdout
    return [l for l in out.splitlines() if l.startswith("R:")]


def answer_regs(line):
    """The answer registers our side put on the wire, as 16-bit words."""
    r = line.split()[0][2:]
    if r == "-":
        return None
    count = int(r[4:6], 16)
    return [int(r[6 + 4 * i:10 + 4 * i], 16) for i in range(count // 2)]


CASES = []


def case(name):
    def deco(fn):
        CASES.append((name, fn))
        return fn
    return deco


@case("a frame we cannot answer gets no answer at all")
def _():
    out = run([poll(0x10),
               wrap(bytes([SLAVE, 0x03, 0x9C, 0xB9, 0x00, 0x08])),          # read registers
               wrap(bytes([SLAVE, 0x06, 0x9C, 0xB9, 0x12, 0x34])),          # write one register
               wrap(bytes([SLAVE, 0x17, 0x9C, 0xBA, 0x00, 0x08,             # wrong read block
                           0x9C, 0x41, 0x00, 0x02, 0x04, 0x11, 0x03, 0, 0]))])
    assert answer_regs(out[0]) is not None, "the ordinary poll must be answered"
    for i, what in ((1, "read registers"), (2, "write one register"), (3, "wrong block")):
        assert answer_regs(out[i]) is None, "%s must not be answered" % what


@case("a lost answer sends the command again, exactly once")
def _():
    out = run([poll(0x1f), "C%d" % OPEN, poll(0x20), poll(0x20), poll(0x21), poll(0x22)])[1:]
    first, repeat, after, later = (answer_regs(o) for o in out)
    assert first[2] == 0x0110, "the press has to go out: %04x" % first[2]
    assert repeat[2] == 0x0110, "the repeated counter has to repeat it: %04x" % repeat[2]
    assert after[2] == 0x0000, "and then stop: %04x" % after[2]
    assert later[2] == 0x0000, "and stay stopped: %04x" % later[2]


@case("a delivered command is not sent a second time")
def _():
    out = run([poll(0x2f), "C%d" % CLOSE, poll(0x30), poll(0x31), poll(0x31), poll(0x31)])[1:]
    sent, _moved, lost1, lost2 = (answer_regs(o) for o in out)
    assert sent[2] == 0x0120, "the press has to go out: %04x" % sent[2]
    for r in (lost1, lost2):
        assert r[2] == 0x0000, "a later loss must not repeat it: %04x" % r[2]


@case("a counter that jumps does not start a repeat storm")
def _():
    seq = [poll(0x3f), "C%d" % OPEN, poll(0x40), poll(0x41)]
    seq += [poll(c) for c in range(0x50, 0x60)]
    out = run(seq)
    presses = sum(1 for o in out if (answer_regs(o) or [0, 0, 0])[2] == 0x0110)
    assert presses == 1, "one press asked for, %d went out" % presses


@case("no command is armed before the drive has ever spoken")
def _():
    out = run(["C%d" % OPEN, poll(0x60), poll(0x61), poll(0x62)])
    for o in out:
        r = answer_regs(o)
        assert r[2] != 0x0110, "a refused press must never reach the wire: %04x" % r[2]


@case("light is one command per direction")
def _():
    out = run([poll(0x70), "C%d" % LAMPON, poll(0x71)])
    on = answer_regs(out[1])
    assert (on[2], on[3]) == (0x0880, 0x0000), "light on: %04x %04x" % (on[2], on[3])


@case("the identity request rides along and the serial arrives in two halves")
def _():
    out = run([poll(0x01), poll(0x02),
               transfer(0x80 | 0x03, 0x0C, b"ABCDEFGHIJKLMN"),
               transfer(0x04, 0x0C, b"OPQRSTUVWXYZ")])
    asked = [answer_regs(o) for o in out[:2]]
    assert any(r[1] & 0xFF == 0x22 and r[2] >> 8 == 0x05 for r in asked), \
        "the serial number has to be asked for: %s" % [hex(r[1]) for r in asked]
    for i in (2, 3):
        r = answer_regs(out[i])
        assert r[1] & 0xFF == 0xFD, "each half has to be acknowledged: %04x" % r[1]


@case("a payload we cannot use is refused, not acknowledged")
def _():
    # Second half with no first half seen: acknowledging it would tell the drive
    # it landed, and it would never send it again.
    out = run([poll(0x01), poll(0x02), transfer(0x05, 0x0C, b"OPQRSTUVWXYZ")])
    r = answer_regs(out[2])
    assert r[1] & 0xFF == 0xFE, "expected a refusal, got %04x" % r[1]


@case("an unknown sub code is refused")
def _():
    out = run([poll(0x01), transfer(0x02, 0x77, b"\x00\x00")])
    r = answer_regs(out[1])
    assert r[1] & 0xFF == 0xFE, "expected a refusal, got %04x" % r[1]


@case("a stop cancels a press that has not gone out yet")
def _():
    # The door is closed, so the stop needs no impulse: cancelling the queued
    # open is the stop. Nothing at all may reach the wire.
    out = run([poll(0x80), "C%d" % OPEN, "S", poll(0x81), poll(0x82)])[1:]
    for o in out:
        r = answer_regs(o)
        assert r[2] != 0x0110, "the cancelled press reached the wire: %04x" % r[2]


@case("a press the drive never fetched goes stale instead of waiting")
def _():
    # The drive keeps the link alive with frames that carry no command, so the
    # press is never fetched. It must not fire minutes later.
    seq = [poll(0x90), "C%d" % OPEN, "T2000"]
    # Frames that refresh the link but do not drain the slot.
    seq += [poll(0x91, command=0x02, read_cnt=5, payload=(0x00, 0x00, 0x00, 0x00))]
    seq += ["T9000", poll(0x92), poll(0x93)]
    out = run(seq)
    for o in out:
        r = answer_regs(o)
        if r is None:
            continue
        assert r[2] != 0x0110, "a stale press reached the wire: %04x" % r[2]


def main():
    failed = 0
    for name, fn in CASES:
        try:
            fn()
            print("  ok    %s" % name)
        except AssertionError as e:
            print("  FAIL  %s\n          %s" % (name, e))
            failed += 1
        except Exception as e:  # a case that cannot even run is a failure
            print("  ERROR %s\n          %r" % (name, e))
            failed += 1
    print("  %d of %d behaviours hold" % (len(CASES) - failed, len(CASES)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
