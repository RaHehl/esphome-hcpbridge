# Long-run check: the command state machine across the millis() wraparound.
# Same command sequence twice - once at an ordinary time, once straddling
# 2**32. The emitted command bytes must be identical.
import struct
def crc(d):
    c=0xFFFF
    for b in d:
        c^=b
        for _ in range(8): c=(c>>1)^0xA001 if c&1 else c>>1
    return c
def h(p): return (p+bytes([crc(p)&0xFF,crc(p)>>8])).hex()
def poll(L): L.append(h(bytes([2,0x17])+struct.pack('>HHHHB',0x9CB9,8,0x9C41,2,4)+b'\x00\x00\x00\x00'))
def run(L, t0):
    for cmd in range(7):
        t=t0
        L.append("T%d"%(t % 2**32)); L.append("C%d"%cmd)
        poll(L)                     # start values
        t+=50;  L.append("T%d"%(t % 2**32)); poll(L)   # still inside the 100 ms
        t+=150; L.append("T%d"%(t % 2**32)); poll(L)   # past it -> end values
        t+=200; L.append("T%d"%(t % 2**32)); poll(L)
        t0 += 1000
A=[];  run(A, 1000)                 # ordinary time
# The clock must wrap INSIDE the 100 ms window, otherwise nothing is tested.
B=[];  run(B, 2**32 - 150)
open('frames5a.txt','w').write('\n'.join(A)+'\n')
open('frames5b.txt','w').write('\n'.join(B)+'\n')
print(len([x for x in A if x[0] not in 'TC']),"frames per run")
