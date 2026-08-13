#!/bin/sh
# Builds the accessory for the host. The component sources are compiled where
# they live, not from a copy, so the simulator can never test a stale duplicate.
#
# SAN=1 adds the address and undefined-behaviour sanitizers, which is how the
# malformed-frame corpus is meant to be run: without them a read past a buffer
# is only caught when it happens to change an answer.
set -e
cd "$(dirname "$0")"
SRC=../../components/hcpbridge
STUB=../stubs

OUT=accessory
F="-std=gnu++17 -I $STUB -I $SRC"
if [ -n "$SAN" ]; then
  F="$F -fsanitize=address,undefined -fno-omit-frame-pointer -g"
  OUT=accessory-san
fi

rm -f ./*.o
g++ $F -c "$SRC/hcp2_codec.cpp" -o sim_codec.o
g++ $F -c "$SRC/modbus_rtu.cpp" -o sim_modbus.o
g++ $F -c "$SRC/hcp_events.cpp" -o sim_events.o
g++ $F -c "$SRC/hcp_state.cpp"  -o sim_state.o
g++ $F -c "$SRC/hcp2_frame.cpp" -o sim_frame.o
g++ $F -c sim_main.cpp          -o sim_main.o
g++ $F -o "$OUT" sim_main.o sim_codec.o sim_modbus.o sim_events.o sim_state.o sim_frame.o
rm -f ./*.o
