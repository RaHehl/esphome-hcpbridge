"""The UART table has to cover every chip ESPHome knows.

A variant missing from the table is a configuration this component refuses on a
board it could serve. Checking that belongs in a test and not in an assert at
import: the assert would stop the component loading for everyone the day
ESPHome learns a new chip, including the people who own none of them.

Run from the config gate, which is the one that installs a real ESPHome. It
refuses to skip: a gate that quietly passes when its dependency is missing is
worse than no gate, because it reports green either way.
"""

import sys

ROOT = __file__.rsplit("/test/", 1)[0]
sys.path.insert(0, ROOT)


def main() -> int:
    try:
        from esphome.components.esp32.const import VARIANTS
    except ImportError:
        print("  FAILED  ESPHome is not importable, so the variant list is unknown")
        print("          run this through test/config/run.sh, which installs it")
        return 1

    try:
        from components.hcpbridge import HP_UART_COUNT
    except ImportError as exc:
        print(f"  FAILED  the component does not import: {exc}")
        return 1

    ours = set(HP_UART_COUNT)
    theirs = set(VARIANTS)

    missing = sorted(theirs - ours)
    extra = sorted(ours - theirs)
    if missing:
        print(f"  FAILED  ESPHome knows chips the UART table does not: {missing}")
        print("          add them to HP_UART_COUNT with their SOC_UART_HP_NUM")
    if extra:
        print(f"  FAILED  the UART table names chips ESPHome does not: {extra}")
    if missing or extra:
        return 1

    print(f"  ok    the UART table covers all {len(theirs)} variants ESPHome knows")
    return 0


if __name__ == "__main__":
    sys.exit(main())
