import random, struct
random.seed(4)
def crc(d):
    c=0xFFFF
    for b in d:
        c^=b
        for _ in range(8): c=(c>>1)^0xA001 if c&1 else c>>1
    return c
def h(p): return (p+bytes([crc(p)&0xFF,crc(p)>>8])).hex()
L=[]
def poll():   # the drive's command poll
    L.append(h(bytes([2,0x17])+struct.pack('>HHHHB',0x9CB9,8,0x9C41,2,4)+struct.pack('>HH',random.getrandbits(16),random.getrandbits(16))))
def empty():
    L.append(h(bytes([2,0x17])+struct.pack('>HHHHB',0x9CB9,2,0x9C41,2,4)+b'\x00\x00\x00\x00'))
def bcast():
    r=[random.getrandbits(16) for _ in range(9)]
    L.append(h(bytes([0,0x10])+struct.pack('>HHB',0x9D31,9,18)+struct.pack('>9H',*r)))
t=1000
for run in range(400):
    cmd=run%7
    L.append("T%d"%t); L.append("C%d"%cmd)
    for step in range(random.randint(1,6)):
        # time steps right around the 100 ms threshold
        t += random.choice([0,1,50,99,100,101,150,1000])
        L.append("T%d"%t)
        random.choice([poll,poll,poll,empty,bcast])()
    t += random.choice([0,100,101])
    L.append("T%d"%t); poll(); poll()
open('frames4.txt','w').write('\n'.join(L)+'\n')
print(len([x for x in L if not x[0] in 'TC']),"frames,",len(L),"lines")
