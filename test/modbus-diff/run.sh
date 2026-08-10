#!/bin/sh
# Differenztest: die abgeloeste Arduino-Fassung gegen die jetzige.
#
# Links laeuft der ECHTE alte Code: modbus-esp8266 plus die hoermann.cpp aus
# dem Commit vor der Umstellung. Rechts der aktuelle Stand. Beide bekommen
# dieselben Telegramme und werden ueber Antwortbytes, alle 20 Register und den
# Geraetezustand verglichen.
set -e
cd "$(dirname "$0")"
PRE=${PRE:-f8a6440^}   # letzter Commit mit der Arduino-Bibliothek
[ -d mbesp ] || git clone -q --depth 1 https://github.com/emelianov/modbus-esp8266.git mbesp
git -C ../.. show "$PRE:components/hcpbridge/hoermann.h"   > old/hoermann.h
git -C ../.. show "$PRE:components/hcpbridge/hoermann.cpp" > old/orig_hoermann.cpp
cp ../../components/hcpbridge/hoermann.h ../../components/hcpbridge/hoermann.cpp \
   ../../components/hcpbridge/modbus_rtu.h ../../components/hcpbridge/modbus_rtu.cpp new/

D="-std=gnu++17 -DMODBUS_USE_STL -I stub -I mbesp/src -I old -I ."
g++ $D -c mbesp/src/Modbus.cpp -o Modbus.o
g++ $D -c old/orig_hoermann.cpp -o orig_hoermann.o
g++ $D -c old/old_side.cpp -o old_side.o
g++ $D -c old/old_main.cpp  -o old_main.o
g++ -o old_bin old_main.o old_side.o orig_hoermann.o Modbus.o

N="-std=gnu++17 -I nstub -I new -I ."
g++ $N -c new/hoermann.cpp   -o new_hoermann.o
g++ $N -c new/modbus_rtu.cpp -o new_modbus.o
g++ $N -c new/new_main.cpp   -o new_main.o
g++ -o new_bin new_main.o new_hoermann.o new_modbus.o

python3 gen2.py
./old_bin < frames2.txt > o_alt.txt
./new_bin < frames2.txt > o_neu.txt
n=$(paste -d'|' o_alt.txt o_neu.txt | awk -F'|' '$1!=$2' | wc -l | tr -d ' ')
echo "vollstaendige Telegramme: $n Abweichung(en)"
[ "$n" = "0" ] || { diff o_alt.txt o_neu.txt | head -20; exit 1; }
