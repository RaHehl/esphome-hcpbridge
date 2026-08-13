#!/bin/sh
# Pieces small enough to test on their own, without a drive on the other end.
set -e
cd "$(dirname "$0")"
SRC=../../components/hcpbridge
F="-std=gnu++17 -I $SRC -pthread"

# The protocol path is meant to hold no allocation and no growing string, and
# nothing about the language enforces that. Grep is crude but it fails loudly
# the moment somebody reaches for the convenient thing.
echo "the protocol path allocates nothing:"
found=$(grep -nE '\bnew\b|std::string|malloc|strdup|std::vector' \
        "$SRC/hcp2_codec.h" "$SRC/hcp2_codec.cpp" "$SRC/hcp_events.h" "$SRC/hcp_events.cpp" \
        "$SRC/hcp1_frame.h" "$SRC/hcp1_frame.cpp" \
        "$SRC/hcp1_codec.h" "$SRC/hcp1_codec.cpp" "$SRC/hcp_state.h" "$SRC/hcp_state.cpp" \
        | grep -vE '^\S+: *(//|\*)' || true)
if [ -n "$found" ]; then
  echo "$found"
  echo "  FAILED: the path is supposed to stay allocation free"
  exit 1
fi
echo "  ok    no allocation, no growing string"
echo

# The acceptance criterion from the plan: a codec builds on a desk with nothing
# faked. Not a convenience - it is what makes "this answers correctly" a thing
# a test can establish, rather than something only a door can.
# Freestanding in the sense the plan meant it: no platform, no exceptions, no
# runtime type information, and nothing pulled in behind our backs. The last
# check is the one that catches a convenience header sneaking in three levels
# down, where nobody would look.
# Reaching past an access specifier makes a test pass by rearranging the thing
# it is testing. The one harness that had to went with the differential test.
# Written down in .clang-format rather than carried as folklore. Skipped when
# the tool is absent so a contributor without it is not stopped; CI has it and
# does not skip.
echo "the sources are formatted:"
FMT=$(command -v clang-format || true)
[ -n "$FMT" ] || FMT=$(ls ../../.esphome-venv/bin/clang-format 2>/dev/null || true)
if [ -n "$FMT" ]; then
  SOURCES=$(git -C ../.. ls-files "*.cpp" "*.h" | sed "s|^|../../|")
  if out=$("$FMT" --dry-run --Werror $SOURCES 2>&1); then
    echo "  ok    every source matches .clang-format"
  else
    echo "$out" | head -5
    echo "  FAILED: run clang-format -i over components/hcpbridge"
    exit 1
  fi
else
  echo "  --    clang-format is not installed, skipped"
fi
echo

# The Python half is half the component, and until now nothing held it to
# anything. Same arrangement as clang-format above: skipped when absent, not
# skipped in CI.
echo "the Python matches ESPHome's rules:"
RUFF=$(command -v ruff || true)
[ -n "$RUFF" ] || RUFF=$(ls ../../.esphome-venv/bin/ruff 2>/dev/null || true)
if [ -n "$RUFF" ]; then
  PY_SOURCES=$(git -C ../.. ls-files "*.py" | sed "s|^|../../|")
  if out=$("$RUFF" check $PY_SOURCES 2>&1 && "$RUFF" format --check $PY_SOURCES 2>&1); then
    echo "  ok    ruff is happy with every .py"
  else
    echo "$out" | head -6
    echo "  FAILED: run ruff check --fix and ruff format"
    exit 1
  fi
else
  echo "  --    ruff is not installed, skipped"
fi
echo

echo "every entity is in the build example:"
python3 entity_coverage.py || exit 1
echo

echo "nothing reaches past private:"
offenders=$(grep -rl --include="*.cpp" --include="*.h" "define private public" ../../test || true)
if [ -n "$offenders" ]; then
  echo "$offenders"
  echo "  FAILED: a harness reached past private instead of using the public surface"
  exit 1
fi
echo "  ok    every harness uses the public surface"
echo

echo "the core builds with nothing faked:"
for unit in hcp_state hcp_events hcp1_frame hcp1_codec hcp2_frame hcp2_codec; do
  g++ -std=gnu++17 -I "$SRC" -fno-exceptions -fno-rtti -fsyntax-only "$SRC/$unit.cpp" \
    || { echo "  FAILED: $unit needs something that is not on a desk"; exit 1; }
  pulled=$(g++ -std=gnu++17 -I "$SRC" -H -fsyntax-only "$SRC/$unit.cpp" 2>&1 \
           | grep -c "esphome/\|driver/uart\|freertos" || true)
  [ "$pulled" = "0" ] \
    || { echo "  FAILED: $unit reaches a platform header, $pulled of them"; exit 1; }
  echo "  ok    $unit"
done
echo

# The codecs above build with nothing faked, and that is the point of them. The
# halves that touch the port cannot, so nothing here compiled them at all - and
# the older bus went to a drive nobody has with an undeclared constant in it.
# Only the build matrix would have caught that, and the build matrix needs a
# push. This is the cheap half of it, on a desk.
echo "both buses still compile:"
for unit in hcp2_transport hcp1_transport modbus_rtu hcp1_serial hcp_bus; do
  g++ -std=gnu++17 -I "$SRC" -I ../stubs -fsyntax-only "$SRC/$unit.cpp" \
    || { echo "  FAILED: $unit does not compile"; exit 1; }
  echo "  ok    $unit"
done
echo

echo "HCP1 frames:"
g++ $F hcp1_frame_test.cpp "$SRC/hcp1_frame.cpp" -o hcp1_frame_test
./hcp1_frame_test
echo

echo "HCP1 exchange:"
g++ $F hcp1_codec_test.cpp "$SRC/hcp1_frame.cpp" "$SRC/hcp1_codec.cpp" \
       "$SRC/hcp_state.cpp" "$SRC/hcp_events.cpp" -o hcp1_codec_test
./hcp1_codec_test
echo

echo "event ring:"
g++ $F event_ring_test.cpp "$SRC/hcp_events.cpp" -o event_ring_test
./event_ring_test

echo
echo "the same under the thread sanitizer:"
g++ $F -fsanitize=thread -g event_ring_test.cpp "$SRC/hcp_events.cpp" -o event_ring_test_tsan
./event_ring_test_tsan

rm -f event_ring_test event_ring_test_tsan hcp1_frame_test hcp1_codec_test
