import random, struct
random.seed(20260810)
def crc(d):
    c=0xFFFF
    for b in d:
        c^=b
        for _ in range(8): c=(c>>1)^0xA001 if c&1 else c>>1
    return c
def adu(payload, good=True):
    c=crc(payload)
    if not good: c^=0x0001
    return payload+bytes([c&0xFF,c>>8])
F=[]
def add(p, good=True): F.append(adu(p,good).hex())

# --- echte HCP-Muster ---
for i in range(200):
    d=random.getrandbits(16); e=random.getrandbits(16)
    # Befehlsabfrage: schreibe 2 an 0x9C41, lies 8 ab 0x9CB9
    add(bytes([2,0x17])+struct.pack('>HHHHB',0x9CB9,8,0x9C41,2,4)+struct.pack('>HH',d,e))
    # leerer Befehl: lies 2
    add(bytes([2,0x17])+struct.pack('>HHHHB',0x9CB9,2,0x9C41,2,4)+struct.pack('>HH',d,e))
    # Busscan: schreibe 3, lies 5
    add(bytes([2,0x17])+struct.pack('>HHHHB',0x9CB9,5,0x9C41,3,6)+struct.pack('>HHH',d,e,d^e))
    # Rundruf Zustand: 9 Register ab 0x9D31
    regs=[random.getrandbits(16) for _ in range(9)]
    add(bytes([0,0x10])+struct.pack('>HHB',0x9D31,9,18)+struct.pack('>9H',*regs))
    add(bytes([2,0x10])+struct.pack('>HHB',0x9D31,9,18)+struct.pack('>9H',*regs))

# --- gezielte Grenzfaelle ---
edge_addrs=[0x0000,0x9C40,0x9C41,0x9C43,0x9C44,0x9CB8,0x9CB9,0x9CC0,0x9CC1,0x9D30,0x9D31,0x9D39,0x9D3A,0xFFFE,0xFFFF]
for fc in (0x17,0x10,0x03,0x06,0x01,0x02,0x04,0x05,0x0F,0x16,0x14,0x15,0x08,0x2B,0x00,0x7F):
    for a in edge_addrs:
        for cnt in (0,1,2,3,5,8,9,0x7D,0x7E,0x80):
            if fc==0x17:
                for wa in edge_addrs[:8]:
                    for wc in (0,1,2,3,9):
                        p=bytes([2,fc])+struct.pack('>HHHHB',a,cnt,wa,wc,(2*wc)&0xFF)+bytes(2*wc)
                        add(p)
            elif fc==0x10:
                p=bytes([2,fc])+struct.pack('>HHB',a,cnt,(2*cnt)&0xFF)+bytes(min(2*cnt,60))
                add(p)
            else:
                p=bytes([2,fc])+struct.pack('>HH',a,cnt)
                add(p)

# --- Zufall: beliebige Laengen, Funktionscodes, Adressen ---
for i in range(20000):
    n=random.randint(1,30)
    p=bytes([random.choice([0,1,2,3,247])])+bytes([random.choice([0x17,0x10,0x03,0x06,random.getrandbits(8)])])+bytes(random.getrandbits(8) for _ in range(n))
    add(p, good=random.random()>0.1)

# --- falsche Byteanzahl / widerspruechliche Felder ---
for i in range(3000):
    cnt=random.randint(0,10); bc=random.randint(0,30); n=random.randint(0,24)
    add(bytes([2,0x17])+struct.pack('>HHHHB',random.choice(edge_addrs),cnt,random.choice(edge_addrs),random.randint(0,10),bc)+bytes(n))
    add(bytes([2,0x10])+struct.pack('>HHB',random.choice(edge_addrs),cnt,bc)+bytes(n))

open('frames.txt','w').write('\n'.join(F)+'\n')
print(len(F),"frames")
