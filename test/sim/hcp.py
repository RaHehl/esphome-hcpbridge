"""Frames on the HCP2 bus, built and read the way the drive does it.

This is the half of the simulator that knows the wire. It is deliberately
separate from the drive's behaviour in drive.py: a mistake here shows up as
every case failing at once, a mistake there as one case failing, and the two
are worth being able to tell apart.

Nothing here is derived from our own implementation. The register blocks,
command values and answer codes come from the protocol notes; where a value
could only be read off our own code it is not asserted on.
"""

SLAVE = 0x02
REG_RESP, REG_CMD, REG_BCAST = 0x9CB9, 0x9C41, 0x9D31

# What the drive asks for, in the low byte of the first register it writes.
CMD_CONNECT, CMD_STATUS, CMD_TRANSFER = 0x02, 0x03, 0x04

# What we answer with, in the low byte of the second register.
RESP_STATUS, RESP_REQUEST, RESP_PAUSE = 0x01, 0x22, 0x29
RESP_ACK, RESP_NAK = 0xFD, 0xFE

# Identity exchange.
REQ_SERIAL, REQ_FIRMWARE = 0x05, 0x06
SUB_SERIAL, SUB_FIRMWARE, SUB_PAUSE_ACK = 0x0C, 0x0D, 0x19

# The drive's state word, high byte of regBcast[2]. Confirmed twice over, by
# our own reading of the bus and independently by the firmware of another
# accessory on the same bus.
ST_STOPPED, ST_OPENING, ST_CLOSING = 0x00, 0x01, 0x02
ST_OPEN, ST_CLOSED = 0x20, 0x40

# The named positions, confirmed at a door: asked to vent and then to open
# half, the drive reported each journey and then each arrival, in that order.
# Note that 0x09 and 0x0A both have low bits set while only 0x09 is a journey,
# so the other accessory's rule of thumb - any low bit means moving - does not
# hold here. A running door outranks a rule read out of somebody else's
# firmware.
ST_MOVE_VENTING = 0x09
ST_VENT = 0x0A
ST_MOVE_HALF = 0x05
ST_HALFOPEN = 0x80

# Door commands, as they appear in the two payload registers we answer with.
CMD_OPEN = (0x0110, 0x0000)
CMD_CLOSE = (0x0120, 0x0000)
CMD_IMPULSE = (0x0140, 0x0000)
CMD_HALF = (0x0100, 0x0400)
CMD_VENT = (0x0100, 0x4000)
CMD_LIGHT_ON = (0x0880, 0x0000)
CMD_LIGHT_OFF = (0x0800, 0x0100)
CMD_NONE = (0x0000, 0x0000)

COMMAND_NAMES = {
    CMD_OPEN: "open",
    CMD_CLOSE: "close",
    CMD_IMPULSE: "impulse",
    CMD_HALF: "half",
    CMD_VENT: "vent",
    CMD_LIGHT_ON: "light on",
    CMD_LIGHT_OFF: "light off",
    CMD_NONE: "nothing",
}


def crc16(data):
    c = 0xFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c


def wrap(body):
    c = crc16(body)
    return bytes(body) + bytes([c & 0xFF, c >> 8])


def status_poll(counter, command=CMD_STATUS, payload=(0x00, 0x00), read_cnt=8):
    # Function code 0x17 reads and writes in one exchange, which is why the
    # drive can poll us and collect our answer without a second frame.
    regs = 1 + len(payload) // 2
    body = bytes(
        [
            SLAVE,
            0x17,
            REG_RESP >> 8,
            REG_RESP & 0xFF,
            0x00,
            read_cnt,
            REG_CMD >> 8,
            REG_CMD & 0xFF,
            0x00,
            regs,
            2 + len(payload),
            counter,
            command,
        ]
    ) + bytes(payload)
    return wrap(body)


def transfer(counter, sub, payload):
    regs = 2 + len(payload) // 2
    body = bytes(
        [
            SLAVE,
            0x17,
            REG_RESP >> 8,
            REG_RESP & 0xFF,
            0x00,
            8,
            REG_CMD >> 8,
            REG_CMD & 0xFF,
            0x00,
            regs,
            2 * regs,
            counter,
            CMD_TRANSFER,
            sub,
            0x00,
        ]
    ) + bytes(payload)
    return wrap(body)


def broadcast(state_hi, position=0, target=0, reg6=0x0000):
    payload = bytearray(18)
    payload[2] = target & 0xFF
    payload[3] = position & 0xFF
    payload[4] = state_hi
    payload[12] = (reg6 >> 8) & 0xFF
    payload[13] = reg6 & 0xFF
    body = bytes([0x00, 0x10, REG_BCAST >> 8, REG_BCAST & 0xFF, 0x00, 9, 18]) + bytes(
        payload
    )
    return wrap(body)


class Reported:
    """What the bridge says the door is doing, as the entities would read it.

    The harness prints this after every exchange and nothing looked at it, so
    the whole reporting half of the codec - the state word turned into a state,
    the register turned into a position - was untested. A mutation halving the
    position passed every gate in this repository.
    """

    def __init__(self, field, name=None):
        self.name = name
        parts = field.split(",")
        self.valid = parts[0] == "1"
        self.state = int(parts[1])
        self.target = float(parts[2])
        self.position = float(parts[3])
        self.light = parts[4] == "1"


class Answer:
    def __init__(self, raw, reported=None):
        self.raw = raw
        self.reported = reported
        self.regs = []
        if raw and raw != "-":
            count = int(raw[4:6], 16)
            self.regs = [
                int(raw[6 + 4 * i : 10 + 4 * i], 16) for i in range(count // 2)
            ]

    @property
    def silent(self):
        return not self.regs

    @property
    def code(self):
        return self.regs[1] & 0xFF if len(self.regs) > 1 else None

    @property
    def command(self):
        if len(self.regs) < 4:
            return CMD_NONE
        return (self.regs[2], self.regs[3])

    @property
    def command_name(self):
        return COMMAND_NAMES.get(
            self.command, "unknown {:04x} {:04x}".format(*self.command)
        )

    @property
    def counter(self):
        return self.regs[0] >> 8 if self.regs else None

    def __repr__(self):
        if self.silent:
            return "<silent>"
        return f"<code {self.code or 0:02x}, {self.command_name}>"
