import random, struct, sys
random.seed(11)
def crc(d):
    c=0xFFFF
    for b in d:
        c^=b
        for _ in range(8): c=(c>>1)^0xA001 if c&1 else c>>1
    return c
def h(p): return (p+bytes([crc(p)&0xFF,crc(p)>>8])).hex()
A=[0x0000,0x9C41,0x9C43,0x9CB9,0x9CC0,0x9D31,0x9D39,0xFFFF]
cat={k:[] for k in ['kurz17','kurz10','kurz16','kurz_kopf','datei','sonst']}
for i in range(800):
    a=random.choice(A); wa=random.choice(A); c1=random.choice([1,2,4,8]); wc=random.choice([1,2,3,9])
    bc=2*wc
    d=bytes(random.getrandbits(8) for _ in range(random.randint(0,bc-1)))
    cat['kurz17'].append(h(bytes([2,0x17])+struct.pack('>HHHHB',a,c1,wa,wc,bc)+d))
    bc2=2*c1
    d2=bytes(random.getrandbits(8) for _ in range(random.randint(0,bc2-1)))
    cat['kurz10'].append(h(bytes([2,0x10])+struct.pack('>HHB',a,c1,bc2)+d2))
    cat['kurz16'].append(h(bytes([2,0x16])+struct.pack('>H',a)+bytes(random.randint(0,3))))
    cat['kurz_kopf'].append(h(bytes([2,random.choice([0x03,0x06,0x01,0x04,0x05])])+bytes(random.randint(0,3))))
    cat['datei'].append(h(bytes([2,random.choice([0x14,0x15])])+bytes([random.choice([0,7,14,0xF6])])+bytes(random.getrandbits(8) for _ in range(14))))
    cat['sonst'].append(h(bytes([2,random.choice([0x08,0x2B,0x80,0xF0,0xFF])])+bytes(random.getrandbits(8) for _ in range(4))))
for k,v in cat.items():
    open('f_'+k+'.txt','w').write('\n'.join(v)+'\n')
    print(k, len(v))
