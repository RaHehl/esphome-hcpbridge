#!/bin/sh
# Differential test: the replaced Arduino version against the current one.
#
# The left side is the REAL old code: modbus-esp8266 plus the hoermann.cpp from
# the commit before the switch; the right side is the current tree. Only the
# serial port is faked, all decision logic is original code. Both sides get the
# same frames and are compared on response bytes, all 20 registers and state.

set -e
cd "$(dirname "$0")"
PRE=${PRE:-a91b75c}    # last upstream commit that still used the Arduino library
rm -f ./*.o          # stale objects with changed struct layouts would silently mislead
[ -d mbesp ] || git clone -q --depth 1 https://github.com/emelianov/modbus-esp8266.git mbesp
git -C ../.. show "$PRE:components/hcpbridge/hoermann.h"   > old/hoermann.h
git -C ../.. show "$PRE:components/hcpbridge/hoermann.cpp" > old/orig_hoermann.cpp
cp ../../components/hcpbridge/hoermann.h ../../components/hcpbridge/hoermann.cpp \
   ../../components/hcpbridge/modbus_rtu.h ../../components/hcpbridge/modbus_rtu.cpp new/

D="-std=gnu++17 -DMODBUS_USE_STL -I stub -I mbesp/src -I old -I ."
g++ $D -c mbesp/src/Modbus.cpp -o Modbus.o
g++ $D -c mbesp/src/ModbusRTU.cpp -o ModbusRTU.o
g++ $D -c old/orig_hoermann.cpp -o orig_hoermann.o
g++ $D -c old/old_side.cpp -o old_side.o
g++ $D -c old/old_main.cpp  -o old_main.o
g++ -o old_bin old_main.o old_side.o orig_hoermann.o Modbus.o ModbusRTU.o

N="-std=gnu++17 -I nstub -I new -I ."
g++ $N -c new/hoermann.cpp   -o new_hoermann.o
g++ $N -c new/modbus_rtu.cpp -o new_modbus.o
g++ $N -c new/new_main.cpp   -o new_main.o
g++ -o new_bin new_main.o new_hoermann.o new_modbus.o

# 1) CRC: the real crc16() of both sides against each other
awk 'NR>=11' mbesp/src/ModbusRTU.cpp | sed -n '1,/};/p' | sed 's/ PROGMEM//' > auchcrc.inc
g++ -std=gnu++17 -I ../../components/hcpbridge -I nstub -I . crc_test.cpp new/modbus_rtu.cpp -o crc_test
./crc_test

compare() {
  python3 "$1" >/dev/null
  ./old_bin < "$2" > o_alt.txt
  ./new_bin < "$2" > o_neu.txt
  n=$(paste -d'|' o_alt.txt o_neu.txt | awk -F'|' '$1!=$2' | wc -l | tr -d ' ')
  t=$(grep -cv '^[TC]' "$2")
  echo "$3: $t frames, $n difference(s)"
  [ "$n" = "0" ] || { diff o_alt.txt o_neu.txt | head -20; exit 1; }
}
# 2) complete frames of every function code, own and foreign slave address
compare gen2.py frames2.txt "function codes"
# 2b) Same frames, but reported split at the driver's RX threshold, which is
#     how frames above RX_FULL_THRESHOLD actually arrive.
CHUNK=120 ./new_bin < frames2.txt > o_neu.txt
n=$(paste -d'|' o_alt.txt o_neu.txt | awk -F'|' '$1!=$2' | wc -l | tr -d ' ')
echo "function codes, split at RX threshold: $n difference(s)"
[ "$n" = "0" ] || { diff o_alt.txt o_neu.txt | head -10; exit 1; }

# 3) Command path: nextCommand plus time steps around the 100 ms threshold
compare gen4.py frames4.txt "commands"

# 3b) Long run: the command state machine across the millis() wraparound.
#     Same sequence at an ordinary time and straddling 2**32 must emit the
#     same command bytes.
python3 gen5.py >/dev/null
CMD_COL=$(( (3+9+2)*4+3 ))
./new_bin < frames5a.txt | awk -v c=$CMD_COL '{print substr($2,c,8)}' > w_a.txt
./new_bin < frames5b.txt | awk -v c=$CMD_COL '{print substr($2,c,8)}' > w_b.txt
if diff -q w_a.txt w_b.txt >/dev/null; then
  echo "millis() wraparound: identical"
else
  echo "millis() wraparound: DIFFERS"; diff w_a.txt w_b.txt | head -10; exit 1
fi

# 4) Known, deliberate differences. These frames are shorter than they claim;
#    the old version read past its buffer there, partly using its own CRC
#    bytes as payload. Reported only, not a failure.
echo "--- deliberate differences (truncated frames) ---"
python3 gen3.py >/dev/null
for k in short17 short10 short16 short_header filerec other; do
  ./old_bin < "f_$k.txt" > a.txt; ./new_bin < "f_$k.txt" > b.txt
  n=$(paste -d'|' a.txt b.txt | awk -F'|' '$1!=$2' | wc -l | tr -d ' ')
  echo "  $k: $n of $(wc -l < "f_$k.txt" | tr -d ' ')"
done
