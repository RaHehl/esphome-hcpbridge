"""Break the codec on purpose and see whether any gate notices.

Written to answer one question - may the differential test against the old
implementation be retired - and kept because it turned out to answer a better
one. On its first run it found a defect no gate here saw at all: every test
watched what went out on the wire and none watched what came back, so halving
the reported position passed all of them. That is the kind of thing a green
suite cannot tell you about itself.

A test that passes on broken code is decoration. This is how that gets caught.

Nothing here touches the working tree. Each mutation is applied inside a git
worktree of its own and thrown away afterwards - which also means this measures
the committed state, not what is open in an editor. Commit first, then ask.
"""

from pathlib import Path
import re
import shutil
import subprocess
import sys

REPO = Path(__file__).resolve().parents[2]
SRC = "components/hcpbridge"

# Each one is a defect somebody could plausibly introduce: a check dropped, a
# constant mistyped, a line lost in a merge. They have to compile - a mutation
# that does not build proves nothing about the tests.
MUTATIONS = [
    {
        "name": "a broken checksum is answered anyway",
        "file": "hcp2_frame.cpp",
        "find": "  if (got != hcp2Crc16(buf, len - 2))\n    return 0;\n",
        "repl": "",
    },
    {
        "name": "the checksum is computed with the wrong polynomial",
        "file": "hcp2_frame.cpp",
        "find": "0xA001",
        "repl": "0xA003",
    },
    {
        "name": "the counter never advances",
        "file": "hcp2_codec.cpp",
        "find": "    this->advanceCounter();\n",
        "repl": "",
    },
    {
        "name": "a press the drive never collected never goes stale",
        "file": "hcp2_codec.cpp",
        "find": "> CMD_STALE_MS)",
        "repl": "> 0xFFFFFFFFu)",
    },
    {
        "name": "one press is sent for ever",
        "file": "hcp2_codec.cpp",
        "find": "    this->nextCommand.store(HoermannCommand::NONE);\n"
        "    // Start watching for the effect.\n",
        "repl": "    // Start watching for the effect.\n",
    },
    {
        "name": "the light's two directions are swapped",
        "file": "hcp_state.cpp",
        "find": "    {0x0880, 0x0000},  // LIGHT_ON\n    {0x0800, 0x0100},  // LIGHT_OFF\n",
        "repl": "    {0x0800, 0x0100},  // LIGHT_ON\n    {0x0880, 0x0000},  // LIGHT_OFF\n",
    },
    {
        "name": "a stop is never sent at all",
        "file": "hcp2_codec.cpp",
        "find": "    cmd = moving ? HoermannCommand::IMPULSE : HoermannCommand::NONE;\n",
        "repl": "    cmd = HoermannCommand::NONE;\n",
    },
    {
        "name": "the position is reported at half its size",
        "file": "hcp2_codec.cpp",
        "find": "this->state->setCurrentPosition((float) (val & 0x00FF) / 200.0f);",
        "repl": "this->state->setCurrentPosition((float) (val & 0x00FF) / 400.0f);",
    },
    {
        "name": "an open door reads as shut",
        "file": "hcp2_codec.cpp",
        "find": "      case 0x20:\n        this->state->setState(HoermannState::State::OPEN);\n",
        "repl": "      case 0x20:\n        this->state->setState(HoermannState::State::CLOSED);\n",
    },
    # The older bus. Nothing here has ever met a drive, so whether anything
    # would notice a defect in it is the question that matters most.
    {
        "name": "hcp1: a broken checksum is answered anyway",
        "file": "hcp1_frame.cpp",
        "find": "  if (hcp1Crc8(buf, len - 1) != buf[len - 1])\n    return false;\n",
        "repl": "",
    },
    {
        "name": "hcp1: the checksum uses the wrong polynomial",
        "file": "hcp1_frame.cpp",
        "find": "^ 0x07)",
        "repl": "^ 0x09)",
    },
    {
        "name": "hcp1: a frame for another address is answered",
        "file": "hcp1_codec.cpp",
        "find": "  if (f.address != HCP1_ADDR_SELF || f.length < 1)",
        "repl": "  if (f.length < 1)",
    },
    {
        "name": "hcp1: an open door reads as shut",
        "file": "hcp1_codec.cpp",
        "find": "  else if (d0 & HCP1_BC_OPEN)\n    next = HoermannState::OPEN;",
        "repl": "  else if (d0 & HCP1_BC_OPEN)\n    next = HoermannState::CLOSED;",
    },
    {
        "name": "hcp1: vent is asked for with the open bit",
        "file": "hcp1_codec.cpp",
        "find": "    case HoermannCommand::VENT:\n      return HCP1_DO_VENT;",
        "repl": "    case HoermannCommand::VENT:\n      return HCP1_DO_OPEN;",
    },
    {
        "name": "the link never falls silent",
        "file": "hcp_state.h",
        "find": "BUS_SILENCE_MS = 20000;",
        "repl": "BUS_SILENCE_MS = 4000000000u;",
    },
]

GATES = [("unit", "test/unit/run.sh"), ("simulator", "test/sim/run.sh")]


def run_gate(tree, script):
    r = subprocess.run(  # noqa: PLW1510 - a failing gate is the result, not an error
        ["sh", script], cwd=tree, capture_output=True, text=True, check=False
    )
    out = r.stdout + r.stderr
    if re.search(r"^\S+error:", out, re.MULTILINE) or "error: " in out:
        # A mutation that does not compile says nothing about any test.
        return "build error"
    return "caught" if r.returncode != 0 else "survived"


def main():
    work = REPO / ".mutation-worktree"
    if work.exists():
        subprocess.run(
            ["git", "worktree", "remove", "--force", str(work)],
            cwd=REPO,
            capture_output=True,
            check=False,
        )
        shutil.rmtree(work, ignore_errors=True)
    subprocess.run(
        ["git", "worktree", "add", "--detach", "--quiet", str(work), "HEAD"],
        cwd=REPO,
        check=True,
    )
    try:
        return report(work)
    finally:
        subprocess.run(
            ["git", "worktree", "remove", "--force", str(work)],
            cwd=REPO,
            capture_output=True,
            check=False,
        )
        shutil.rmtree(work, ignore_errors=True)


def report(work):
    rows, bad_anchor, survived_all = [], [], []

    for m in MUTATIONS:
        subprocess.run(["git", "checkout", "--quiet", "--", "."], cwd=work, check=True)
        path = work / SRC / m["file"]
        text = path.read_text()
        if text.count(m["find"]) != 1:
            # The source moved and the mutation no longer describes a defect.
            # Reporting a pass here would be reporting that nothing was tested.
            bad_anchor.append(m["name"])
            print(
                "  ANCHOR  {}\n          not found exactly once in {}".format(
                    m["name"], m["file"]
                )
            )
            continue
        path.write_text(text.replace(m["find"], m["repl"]))

        results = {name: run_gate(work, script) for name, script in GATES}
        rows.append((m["name"], results))
        caught = any(results[n] == "caught" for n, _ in GATES)
        mark = "ok   " if caught else "BLIND"
        seen = " ".join(f"{n}={results[n]}" for n, _ in GATES)
        print(f"  {mark} {m['name']:<52} {seen}")
        if not caught:
            survived_all.append(m["name"])

    print()
    if bad_anchor:
        print(
            f"{len(bad_anchor)} mutation(s) no longer describe the code "
            "they were written against."
        )
        return 1
    if survived_all:
        print("Seen by no gate at all - these ship broken and green:")
        for n in survived_all:
            print(f"  - {n}")
        return 1
    print("Every defect was seen by a gate.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
