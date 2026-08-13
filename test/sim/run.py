"""Scenarios and invariants, played out against a modelled drive.

This runs the accessory against a modelled drive and asks whether the exchange
as a whole stays sane: an answer arriving late, a command surviving a fault, a
door reached from the wrong state, a position read back as the one that was
sent. It replaced a differential test against the implementation this work
supersedes, once test/mutation showed it catches everything that one did.

Cases assert on one named situation. Properties are checked over randomised
runs with faults injected, and are the ones that would catch a fault nobody
thought to write a case for.
"""

import sys

import drive
import hcp

BINARY = "./accessory"

CASES = []


def case(name):
    def deco(fn):
        CASES.append((name, fn))
        return fn

    return deco


def fresh(**kw):
    acc = drive.Accessory(BINARY)
    return drive.Drive(acc, **kw)


@case("the drive gets a working accessory without being asked twice")
def _(d):
    d.run(10)
    assert not any(a.silent for a in d.answers), "the drive polled and got nothing back"
    assert "serial" in d.identity_sent, "the serial number was never asked for"
    assert "firmware" in d.identity_sent, "the firmware version was never asked for"


@case("a key press reaches the drive once and moves the door")
def _(d):
    d.run(5)
    d.acc.press(drive.PRESS_OPEN)
    d.run(10)
    assert d.door_commands() == [hcp.CMD_OPEN], "expected exactly one open, got %s" % [
        hcp.COMMAND_NAMES.get(c) for c in d.door_commands()
    ]
    assert d.position > 0, "the door never moved"


@case("a lost answer is repeated, and the door still only moves once")
def _(d):
    d.run(5)
    d.acc.press(drive.PRESS_OPEN)
    d.drop_next_answer = True
    d.run(10)
    sent = d.door_commands()
    assert sent == [hcp.CMD_OPEN], f"one press became {len(sent)} commands"


@case("a press made while the bus is dead does not fire when it comes back")
def _(d):
    # Nothing has been heard yet, so the accessory has no reason to believe
    # anybody is listening.
    d.acc.press(drive.PRESS_OPEN)
    d.run(10)
    assert not d.door_commands(), (
        "a press from before the link existed reached the drive"
    )


@case("a press the drive never collects goes stale instead of waiting")
def _(d):
    d.run(5)
    d.acc.press(drive.PRESS_OPEN)
    # The bus carries nothing for us for long enough that whoever pressed has
    # walked away.
    d.quiet(9000)
    d.run(10)
    assert not d.door_commands(), "a press from nine seconds ago still moved the door"


@case("a stop cancels a press that has not gone out yet")
def _(d):
    d.run(5)
    d.acc.press(drive.PRESS_OPEN)
    d.acc.stop()
    d.run(10)
    assert not d.door_commands(), "the cancelled press reached the drive anyway"


@case("a stop reaches a moving door")
def _(d):
    d.run(5)
    d.acc.press(drive.PRESS_OPEN)
    d.run(6)
    assert d.moving, "the door should be moving by now"
    d.acc.stop()
    d.run(6)
    assert not d.moving, "the door kept going after a stop"


@case("vent and half-open are asked for once and reached")
def _(d):
    d.run(5)
    for press, expected in (
        (drive.PRESS_VENT, hcp.CMD_VENT),
        (drive.PRESS_HALF, hcp.CMD_HALF),
    ):
        before = len([c for c in d.commands_seen if c == expected])
        d.acc.press(press)
        d.run(20)
        after = len([c for c in d.commands_seen if c == expected])
        assert after - before == 1, f"one press produced {after - before} commands"


@case("a link that never existed is told apart from one that fell quiet")
def _(d):
    # The two need opposite responses at the door - check the wiring, or power
    # cycle the drive - and a single connected flag says the same thing to both.
    assert d.acc.link() == "Never seen", "before anything, nothing has been seen"
    d.run(3)
    assert d.acc.link() == "Registered", "answering polls is what registration is"
    d.quiet(30000)
    assert d.acc.link() == "Silent", (
        "and going quiet is its own state, not the first one"
    )
    d.run(3)
    assert d.acc.link() == "Registered", "traffic brings it back"


@case("a stop does not start a door that arrived while it was queued")
def _(d):
    # Press stop while it is moving, then let it reach the end before the drive
    # collects the answer. Decided when the button was pressed, the impulse
    # would go out at a standing door - and an impulse at a standing door is
    # how you open one.
    d.run(5)
    d.acc.press(drive.PRESS_OPEN)
    d.run(4)
    assert d.moving, "the door should be moving by now"
    d.acc.stop()
    # It gets there on its own before the next poll collects anything.
    d.state, d.position = hcp.ST_OPEN, 200.0
    before = len(d.door_commands())
    d.run(6)
    assert len(d.door_commands()) == before, (
        "a stop reached a door that was already standing, which starts it"
    )


@case("the position the bridge reports is the one the drive sent")
def _(d):
    # The reporting half of the codec had no test at all: a mutation halving
    # the position passed every gate in this repository, because all of them
    # watched what goes out and none watched what comes back.
    d.run(4)
    d.acc.press(drive.PRESS_OPEN)
    for _ in range(30):
        d.run(1)
        expected = d.position / 200.0
        assert abs(d.reported.position - expected) < 0.01, (
            f"the drive is at {d.position:.0f} of 200, the bridge says {d.reported.position * 200:.0f}"
        )


@case("every state the drive reports is read back as itself")
def _(d):
    # By name, so this says what it expects instead of repeating the enum's
    # order back at the code under test.
    expected = {
        hcp.ST_CLOSED: "Closed",
        hcp.ST_OPEN: "Open",
        hcp.ST_OPENING: "Opening",
        hcp.ST_CLOSING: "Closing",
        hcp.ST_STOPPED: "Stopped",
        hcp.ST_VENT: "Venting",
        hcp.ST_MOVE_VENTING: "Move venting",
        hcp.ST_HALFOPEN: "Half open",
        hcp.ST_MOVE_HALF: "Move half",
    }
    d.run(3)
    for word, name in expected.items():
        d.frozen, d.state = True, word
        d.run(2)
        assert d.reported.name == name, (
            f"state word 0x{word:02x} came back as {d.reported.name}, not {name}"
        )


@case("the light follows what was asked for")
def _(d):
    d.run(5)
    d.acc.press(drive.PRESS_LIGHT)
    d.run(6)
    assert d.light, "the light never came on"


@case("the accessory keeps answering across a long quiet stretch")
def _(d):
    d.run(5)
    d.quiet(30000)
    answers = d.run(5)
    assert not any(a.silent for a in answers), (
        "the accessory stopped answering after the bus went quiet"
    )


@case("nothing is sent to a drive that has gone quiet for good")
def _(d):
    d.run(5)
    d.quiet(60000)
    d.acc.press(drive.PRESS_OPEN)
    d.run(1)
    # The link is stale, so the press is refused rather than queued: it belongs
    # to a moment that has passed.
    assert not d.door_commands(), "a press was armed against a link long gone"


PROPERTIES = []


def prop(name):
    def deco(fn):
        PROPERTIES.append((name, fn))
        return fn

    return deco


@prop("one press never becomes two door commands")
def _(d, press):
    d.run(4)
    d.acc.press(press)
    d.run(25)
    n = len(d.door_commands())
    assert n <= 1, f"one press produced {n} door commands"


@prop("after a stop, nothing further reaches the drive")
def _(d, press):
    d.run(4)
    d.acc.press(press)
    d.acc.stop()
    before = len(d.door_commands())
    d.run(25)
    after = len(d.door_commands())
    # A stop on a moving door is itself an impulse, so one may appear; what must
    # not happen is the cancelled press arriving afterwards.
    assert after - before <= 1, f"{after - before} commands arrived after a stop"


@prop("every poll is either answered or silent for a reason")
def _(d, press):
    d.run(4)
    d.acc.press(press)
    d.run(25)
    assert not any(a.silent for a in d.answers), (
        "the drive was left without an answer while the link was up"
    )


@case("a scrubbed recording is still a valid exchange")
def _(d):
    # A recording is stripped of the serial number before it is written, and a
    # frame whose checksum was not recomputed would be refused rather than
    # replayed - the recording would then pass by testing nothing.
    import trace

    d.run(4)
    payload = b"REAL-SERIAL-99"
    scrubbed = trace.scrub(hcp.transfer(0x83, hcp.SUB_SERIAL, payload))
    assert payload not in scrubbed, "the serial number survived scrubbing"
    answer = d.acc.exchange(scrubbed)
    assert not answer.silent, "the scrubbed frame was not accepted at all"
    assert answer.code in (hcp.RESP_ACK, hcp.RESP_NAK), (
        f"expected an acknowledgement or a refusal, got code {answer.code}"
    )


DEFECTS = []


def defect(name):
    """A defect that is known, reproduced, and not fixed yet.

    These assert what the code should do, and are expected to fail until the
    commit that closes them. They are reported and do not turn the run red, so
    that the build stays a signal about regressions; a defect that starts
    passing is called out, because that is the moment to promote it to a case.
    """

    def deco(fn):
        DEFECTS.append((name, fn))
        return fn

    return deco


@case("a drive that never reacts is asked at most twice")
def _(d):
    # The confirmation and the deadline are measured from different moments:
    # one from the last send, one from the first. When both ran from the same
    # one, each repeat pushed the deadline out of reach and this went on for as
    # long as the drive stayed unmoved.
    d.run(5)
    # A drive that answers every poll and never acts on any of it. Rare, but it
    # is the case the give-up deadline exists for: a control board that is alive
    # on the bus and stuck behind it.
    d.reacts = False
    d.acc.press(drive.PRESS_OPEN)
    d.run(60)
    sent = [c for c in d.commands_on_the_wire() if c == hcp.CMD_OPEN]
    assert len(sent) <= 2, (
        f"one press went out {len(sent)} times against a drive that never acts"
    )


@case("a refused go-to-position leaves no target behind")
def _(d):
    # The target is armed only after the command it belongs to is accepted.
    # Written first, it survived a refusal and then stopped a later, unrelated
    # run at a position nobody had asked for.
    d.acc.go_to(50)  # refused: the accessory has heard nothing yet
    d.run(6)  # now the link is up
    d.acc.press(drive.PRESS_OPEN)
    d.run(70)  # long enough to travel the full 200
    assert d.position >= 200.0, (
        f"a plain open stopped at {d.position:.0f} of 200, at the target of a request that "
        "was refused"
    )


def run_defects():
    surprises = 0
    for name, fn in DEFECTS:
        d = fresh()
        try:
            fn(d)
            print(f"  FIXED {name}\n          this now holds; promote it to a case")
            surprises += 1
        except AssertionError as e:
            print(f"  known {name}\n          {e}")
        except Exception as e:
            print(f"  ERROR {name}\n          {e!r}")
        finally:
            d.acc.close()
    return surprises


def run_cases():
    failed = 0
    for name, fn in CASES:
        d = fresh()
        try:
            fn(d)
            print(f"  ok    {name}")
        except AssertionError as e:
            print(f"  FAIL  {name}\n          {e}")
            failed += 1
        except Exception as e:
            print(f"  ERROR {name}\n          {e!r}")
            failed += 1
        finally:
            d.acc.close()
    return failed


def run_properties(rounds=6):
    presses = [
        drive.PRESS_OPEN,
        drive.PRESS_CLOSE,
        drive.PRESS_IMPULSE,
        drive.PRESS_HALF,
        drive.PRESS_VENT,
        drive.PRESS_LIGHT,
    ]
    failed = 0
    for name, fn in PROPERTIES:
        bad = None
        for seed in range(rounds):
            press = presses[seed % len(presses)]
            d = fresh(seed=seed)
            # A fifth of the answers never arrive, which is far worse than any
            # real bus and is the point: the invariant has to hold anyway.
            d.drop_probability = 0.2
            try:
                fn(d, press)
            except AssertionError as e:
                bad = f"seed {seed}, press {press}: {e}"
            except Exception as e:
                bad = f"seed {seed}, press {press}: {e!r}"
            finally:
                d.acc.close()
            if bad:
                break
        if bad:
            print(f"  FAIL  {name}\n          {bad}")
            failed += 1
        else:
            print(f"  ok    {name} ({rounds} rounds with a fifth of answers lost)")
    return failed


def main():
    print("scenarios:")
    failed = run_cases()
    print("invariants:")
    failed += run_properties()
    total = len(CASES) + len(PROPERTIES)
    print(f"  {total - failed} of {total} hold")
    if DEFECTS:
        print("known defects, reproduced and not yet fixed:")
        run_defects()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
