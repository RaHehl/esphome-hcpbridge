#!/bin/sh
# The simulator: the accessory against a modelled drive, then the same binary
# against frames no drive would send, the second time under sanitizers.
set -e
cd "$(dirname "$0")"

sh build.sh
echo "against a modelled drive:"
python3 run.py

echo
echo "malformed and foreign frames:"
python3 fuzz.py 4000

echo
echo "the same, under the address and undefined-behaviour sanitizers:"
SAN=1 sh build.sh
python3 fuzz.py 4000 ./accessory-san

# A recording from a real drive, if one has been made. It is the only input
# here that nobody in this repository wrote, so it is worth failing over, but
# its absence is not a failure: it takes a trip to a door.
if [ -f traces/gate.trace ]; then
  echo
  echo "recorded from a real drive:"
  sh build.sh
  python3 trace.py traces/gate.trace
fi
