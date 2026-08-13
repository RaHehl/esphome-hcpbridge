"""Recorded bus traffic: reading it back, and stripping it before it is kept.

Everything else the simulator knows was written by us, from our own reading of
the bus and of a related firmware. A recording from a real drive is the one
input in this directory whose content nobody here decided, which is what makes
it worth having even though it is only one door.

The drive puts its serial number and firmware version on the wire, so a
recording identifies a specific installation. Those two payloads are replaced
here, at capture time rather than afterwards, so the real ones never reach a
file. The replacement is the same length and the checksum is recomputed:
without that the frames stop parsing and the recording tests nothing.

Format, one frame per line:

    <ms since start> <D|A> <hex>

D is what the drive sent, A is what the accessory answered ("-" for silence).
"""

import pathlib
import sys

import hcp

# Sub codes whose payload names the installation.
IDENTIFYING_SUBS = (hcp.SUB_SERIAL, hcp.SUB_FIRMWARE)

FILLER = b"SCRUBBED-0000000000000000000000"


def scrub(frame):
    """Replace an identity payload, keeping length and checksum consistent.

    Frames that carry nothing identifying come back unchanged, so this is safe
    to run over a whole recording.
    """
    if len(frame) < 15 or frame[1] != 0x17:
        return frame
    body = bytearray(frame[:-2])
    # counter, command, sub code: the payload starts after them.
    if body[12] != hcp.CMD_TRANSFER:
        return frame
    if body[13] not in IDENTIFYING_SUBS:
        return frame
    start = 15
    payload_len = len(body) - start
    if payload_len <= 0:
        return frame
    body[start:] = FILLER[:payload_len].ljust(payload_len, b"0")
    return hcp.wrap(bytes(body))


def carries_identity(frame):
    return scrub(frame) != frame


def read(path):
    frames = []
    for raw in pathlib.Path(path).read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        ms, direction, payload = line.split()
        frames.append(
            (int(ms), direction, None if payload == "-" else bytes.fromhex(payload))
        )
    return frames


def write(path, frames):
    with pathlib.Path(path).open("w", encoding="utf-8") as fh:
        fh.write(
            "# Recorded from a real drive. Serial number and firmware\n"
            "# version are replaced; see trace.py.\n"
        )
        for ms, direction, payload in frames:
            if direction == "D" and payload is not None:
                payload = scrub(payload)
            hexed = "-" if payload is None else payload.hex()
            fh.write(f"{ms} {direction} {hexed}\n")


def replay(path, binary="./accessory"):
    """Feed a recording to the accessory and compare against what was recorded.

    Differences are reported rather than asserted: the recording was made
    against one drive at one moment, and this implementation is allowed to have
    changed on purpose since. What it cannot do is fall silent where the
    recording shows an answer, and that is what fails.
    """
    import drive

    frames = read(path)
    acc = drive.Accessory(binary)
    checked = differing = 0
    silent_now = []
    expected = None
    start = frames[0][0] if frames else 0
    for ms, direction, payload in frames:
        if direction == "D":
            acc.set_clock(1000 + ms - start)
            expected = None
            answer = acc.exchange(payload)
            expected = answer
            continue
        checked += 1
        recorded = payload.hex() if payload is not None else "-"
        got = expected.raw if expected and not expected.silent else "-"
        if got != recorded:
            differing += 1
            if got == "-" and recorded != "-":
                silent_now.append((ms, recorded))
    acc.close()

    print(f"  {checked} recorded answers, {differing} differ")
    for ms, recorded in silent_now[:5]:
        print(f"    SILENT at {ms} ms, the drive was answered then: {recorded}")
    return 1 if silent_now else 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    sys.exit(replay(sys.argv[1], *sys.argv[2:]))
