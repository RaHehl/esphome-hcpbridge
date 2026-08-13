"""A Hoermann drive, played against the accessory under test.

Asks the one question that matters at a door: does this behave like something
the drive will keep talking to.

What is modelled here comes from the protocol notes, not from reading our own
implementation, so that agreeing with it means something. Where the notes are
uncertain the behaviour is left out rather than guessed, and the cases that
would have relied on it are not written.
"""

import random
import subprocess

import hcp

# The drive polls several times a second. Everything timed in the accessory is
# expressed in multiples of this, so it decides how long the scenarios below
# take in modelled time.
POLL_MS = 200

# Position runs 0..200 on the wire. Eight seconds end to end, measured at a
# door: it went from half open to shut in four.
TRAVEL_MS = 8000
STEP = 200 * POLL_MS / TRAVEL_MS


class Accessory:
    def __init__(self, binary):
        self.reported = None
        self.p = subprocess.Popen(
            [binary],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self.sent = []

    def _write(self, line):
        self.p.stdin.write(line + "\n")
        self.p.stdin.flush()

    def set_clock(self, ms):
        self._write(f"T{int(ms)}")

    def press(self, index):
        self._write(f"C{index}")

    def stop(self):
        self._write("S")

    def component_tick(self):
        self._write("U")

    def link(self):
        self._write("L")
        return self.p.stdout.readline().strip()

    def go_to(self, percent):
        self._write(f"P{int(percent)}")

    def exchange(self, frame):
        self._write(frame.hex())
        line = self.p.stdout.readline()
        if not line:
            raise RuntimeError("the accessory stopped answering entirely")
        fields = line.split()
        # "S:" carries what the bridge would publish; it is checked in run.py
        # against what this drive actually reported.
        told = next((f[2:] for f in fields if f.startswith("S:")), None)
        # Everything after N:, not a whitespace field: the names have spaces in
        # them, and half of one silently compares equal to nothing.
        name = line.split("N:", 1)[1].strip() if "N:" in line else None
        answer = hcp.Answer(fields[0][2:], hcp.Reported(told, name) if told else None)
        self.reported = answer.reported
        self.sent.append(answer)
        return answer

    def close(self):
        try:
            self.p.stdin.close()
            self.p.wait(timeout=5)
        except Exception:
            self.p.kill()


# Key presses, in the order the host build exposes them.
PRESS_OPEN, PRESS_CLOSE, PRESS_IMPULSE, PRESS_HALF, PRESS_VENT, PRESS_LIGHT = range(6)


class Drive:
    # Owns the clock: the accessory is told the time before every frame, so a
    # scenario reads as wall time instead of as a number of polls.

    def __init__(
        self,
        accessory,
        seed=0,
        serial=b"SIMULATED-SERIAL-0000000000",
        firmware=b"SIM-FW-01.00",
    ):
        self.acc = accessory
        self.rng = random.Random(seed)
        self.now = 1000
        self.counter = 1
        self.position = 0.0
        self.state = hcp.ST_CLOSED
        self.light = False
        self.serial, self.firmware = serial, firmware

        # Faults, all off unless a scenario turns them on. `reacts = False` is a
        # drive that stays on the bus and answers, but whose door never does
        # anything - which is what the accessory's give-up deadline is for.
        self.drop_next_answer = False
        self.drop_probability = 0.0
        self.reacts = True
        # Holds the door where it is. For asking what a state word is read back
        # as, where letting the model run would move the door out from under
        # the question before it is asked.
        self.frozen = False

        # What the accessory has been told to do, in the order it went out.
        self.commands_seen = []
        self.answers = []
        # Identity halves the accessory has acknowledged.
        self.identity_sent = []

    # Where the door comes to rest once it has travelled to a named position.
    # Vent took about a second from shut, half open about half the travel.
    ARRIVES_AT = {
        hcp.ST_MOVE_VENTING: (hcp.ST_VENT, 25.0),
        hcp.ST_MOVE_HALF: (hcp.ST_HALFOPEN, 100.0),
    }

    def _advance_door(self):
        if self.frozen:
            return
        if self.state == hcp.ST_OPENING:
            self.position = min(200.0, self.position + STEP)
            if self.position >= 200.0:
                self.state = hcp.ST_OPEN
        elif self.state == hcp.ST_CLOSING:
            self.position = max(0.0, self.position - STEP)
            if self.position <= 0.0:
                self.state = hcp.ST_CLOSED
        elif self.state in self.ARRIVES_AT:
            arrived, target = self.ARRIVES_AT[self.state]
            step = STEP if target > self.position else -STEP
            self.position += step
            if (step > 0) == (self.position >= target):
                self.position, self.state = target, arrived

    @property
    def reported(self):
        """What the bridge last said about the door, as an entity would read it."""
        return self.acc.reported

    @property
    def moving(self):
        return self.state in (
            hcp.ST_OPENING,
            hcp.ST_CLOSING,
            hcp.ST_MOVE_VENTING,
            hcp.ST_MOVE_HALF,
        )

    def _apply(self, command):
        if command == hcp.CMD_NONE:
            return
        self.commands_seen.append(command)
        if not self.reacts:
            return
        if command == hcp.CMD_OPEN:
            if self.state != hcp.ST_OPEN:
                self.state = hcp.ST_OPENING
        elif command == hcp.CMD_CLOSE:
            if self.state != hcp.ST_CLOSED:
                self.state = hcp.ST_CLOSING
        elif command == hcp.CMD_IMPULSE:
            # The wall-button behaviour: a moving door stops, a standing one
            # starts in the direction away from where it is.
            if self.moving:
                self.state = hcp.ST_STOPPED
            elif self.position >= 200.0:
                self.state = hcp.ST_CLOSING
            else:
                self.state = hcp.ST_OPENING
        elif command == hcp.CMD_VENT:
            if self.state != hcp.ST_VENT:
                self.state = hcp.ST_MOVE_VENTING
        elif command == hcp.CMD_HALF:
            if self.state != hcp.ST_HALFOPEN:
                self.state = hcp.ST_MOVE_HALF
        elif command == hcp.CMD_LIGHT_ON:
            self.light = True
        elif command == hcp.CMD_LIGHT_OFF:
            self.light = False

    def _next_counter(self):
        self.counter = 1 if self.counter >= 0x7F else self.counter + 1
        return self.counter

    def _lose_this_one(self):
        if self.drop_next_answer:
            self.drop_next_answer = False
            return True
        return self.drop_probability and self.rng.random() < self.drop_probability

    def tick(self):
        self.now += POLL_MS
        self.acc.set_clock(self.now)
        self._advance_door()

        reg6 = 0x0010 if self.light else 0x0000
        self.acc.exchange(
            hcp.broadcast(self.state, int(self.position), target=0, reg6=reg6)
        )

        answer = self.acc.exchange(hcp.status_poll(self.counter))
        self.answers.append(answer)

        if answer.silent:
            # Nothing came back; the drive holds its counter and asks again.
            return answer

        if self._lose_this_one():
            # The answer was sent but never arrived. From the drive's side that
            # is the same as silence: the counter stays where it is, and the
            # command that answer carried was never seen.
            return answer

        # Those same two registers carry the request when the answer is a 0x22,
        # so reading them as a door command there would invent one.
        if answer.code == hcp.RESP_STATUS:
            self._apply(answer.command)
        elif answer.code == hcp.RESP_REQUEST:
            self._serve_identity(answer)
        self._next_counter()
        return answer

    def run(self, cycles):
        return [self.tick() for _ in range(cycles)]

    def quiet(self, ms):
        # The component keeps running while the bus carries nothing for us, and
        # noticing that the link has gone stale is its job, not the frame path's.
        step = 500
        for _ in range(max(1, ms // step)):
            self.now += step
            self.acc.set_clock(self.now)
            self.acc.component_tick()

    def _serve_identity(self, answer):
        wanted = answer.regs[2] >> 8 if len(answer.regs) > 2 else 0
        if wanted == hcp.REQ_SERIAL:
            first, second = self.serial[:14], self.serial[14:26]
            self._transfer(0x80 | self._next_counter(), hcp.SUB_SERIAL, first)
            self._transfer(self._next_counter(), hcp.SUB_SERIAL, second)
            self.identity_sent.append("serial")
        elif wanted == hcp.REQ_FIRMWARE:
            self._transfer(self._next_counter(), hcp.SUB_FIRMWARE, self.firmware)
            self.identity_sent.append("firmware")

    def _transfer(self, counter, sub, payload):
        self.now += POLL_MS
        self.acc.set_clock(self.now)
        return self.acc.exchange(hcp.transfer(counter, sub, payload))

    def door_commands(self):
        door = (
            hcp.CMD_OPEN,
            hcp.CMD_CLOSE,
            hcp.CMD_IMPULSE,
            hcp.CMD_HALF,
            hcp.CMD_VENT,
        )
        return [c for c in self.commands_seen if c in door]

    def commands_on_the_wire(self):
        return [
            a.command
            for a in self.answers
            if not a.silent and a.code == hcp.RESP_STATUS and a.command != hcp.CMD_NONE
        ]
