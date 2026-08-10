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
rm -f ./*.o          # sonst koennen veraltete Objektdateien mit geaenderten Strukturen mitlaufen
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

# 1) Pruefsumme: die echten crc16() beider Seiten gegeneinander
awk 'NR>=11' mbesp/src/ModbusRTU.cpp | sed -n '1,/};/p' | sed 's/ PROGMEM//' > auchcrc.inc
g++ -std=gnu++17 -I ../../components/hcpbridge -I nstub -I . crc_test.cpp new/modbus_rtu.cpp -o crc_test
./crc_test

vergleiche() {
  python3 "$1" >/dev/null
  ./old_bin < "$2" > o_alt.txt
  ./new_bin < "$2" > o_neu.txt
  n=$(paste -d'|' o_alt.txt o_neu.txt | awk -F'|' '$1!=$2' | wc -l | tr -d ' ')
  t=$(grep -cv '^[TC]' "$2")
  echo "$3: $t Telegramme, $n Abweichung(en)"
  [ "$n" = "0" ] || { diff o_alt.txt o_neu.txt | head -20; exit 1; }
}
# 2) vollstaendige Telegramme aller Funktionscodes, eigene und fremde Adresse
vergleiche gen2.py frames2.txt "Funktionscodes"
# 2b) Dasselbe, aber vom Treiber an der Puffermarke gestueckelt gemeldet.
#     So kommen Telegramme ueber RX_FULL_THRESHOLD Byte real an.
CHUNK=120 ./new_bin < frames2.txt > o_neu.txt
n=$(paste -d'|' o_alt.txt o_neu.txt | awk -F'|' '$1!=$2' | wc -l | tr -d ' ')
echo "Funktionscodes, an der Puffermarke gestueckelt: $n Abweichung(en)"
[ "$n" = "0" ] || { diff o_alt.txt o_neu.txt | head -10; exit 1; }

# 3) Befehlsstrecke: nextCommand plus Zeitspruenge um die 100-ms-Schwelle
vergleiche gen4.py frames4.txt "Befehle"

# 4) Bekannte, bewusste Abweichungen. Diese Telegramme sind kuerzer als das,
#    was sie ankuendigen; die alte Fassung las dort ueber den Puffer hinaus,
#    teils die eigenen CRC-Bytes als Nutzdaten. Nur Bericht, kein Fehler.
echo "--- bewusste Abweichungen (abgeschnittene Telegramme) ---"
python3 gen3.py >/dev/null
for k in kurz17 kurz10 kurz16 kurz_kopf datei sonst; do
  ./old_bin < "f_$k.txt" > a.txt; ./new_bin < "f_$k.txt" > b.txt
  n=$(paste -d'|' a.txt b.txt | awk -F'|' '$1!=$2' | wc -l | tr -d ' ')
  echo "  $k: $n von $(wc -l < "f_$k.txt" | tr -d ' ')"
done
