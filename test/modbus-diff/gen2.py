import random, struct
random.seed(7)
def crc(d):
    c=0xFFFF
    for b in d:
        c^=b
        for _ in range(8): c=(c>>1)^0xA001 if c&1 else c>>1
    return c
F=[]
def add(p):
    if len(p)+2 > 256: return   # laengere Telegramme gibt es auf Modbus RTU nicht
    F.append((p+bytes([crc(p)&0xFF,crc(p)>>8])).hex())
A=[0x0000,0x9C40,0x9C41,0x9C42,0x9C43,0x9C44,0x9CB8,0x9CB9,0x9CBC,0x9CC0,0x9CC1,
   0x9D30,0x9D31,0x9D32,0x9D37,0x9D39,0x9D3A,0xFFF0,0xFFFF]
SL=[2,2,2,0,1,3,247]
# echte HCP-Muster
for i in range(300):
    d=random.getrandbits(16); e=random.getrandbits(16)
    add(bytes([2,0x17])+struct.pack('>HHHHB',0x9CB9,8,0x9C41,2,4)+struct.pack('>HH',d,e))
    add(bytes([2,0x17])+struct.pack('>HHHHB',0x9CB9,2,0x9C41,2,4)+struct.pack('>HH',d,e))
    add(bytes([2,0x17])+struct.pack('>HHHHB',0x9CB9,5,0x9C41,3,6)+struct.pack('>HHH',d,e,d^e))
    r=[random.getrandbits(16) for _ in range(9)]
    add(bytes([0,0x10])+struct.pack('>HHB',0x9D31,9,18)+struct.pack('>9H',*r))
    add(bytes([2,0x10])+struct.pack('>HHB',0x9D31,9,18)+struct.pack('>9H',*r))
# vollstaendige Telegramme aller Funktionscodes: Laenge deckt immer die angekuendigte Byteanzahl
for it in range(30000):
    sl=random.choice(SL); fc=random.choice([0x17,0x10,0x03,0x06,0x16,0x01,0x02,0x04,0x05,0x0F,0x08,0x2B,0x41,0x80,0x83,0x90,0xAF,0xDC,0xF0,0xFF,random.getrandbits(8)])
    a=random.choice(A); c1=random.choice([0,1,2,3,4,5,8,9,0x7D,0x7E])
    if fc==0x17:
        wa=random.choice(A); wc=random.choice([0,1,2,3,4,9])
        bc=random.choice([2*wc, 2*wc, (2*wc+2)&0xFF])
        data=bytes(random.getrandbits(8) for _ in range(bc))
        add(bytes([sl,fc])+struct.pack('>HHHHB',a,c1,wa,wc,bc)+data)
    elif fc in (0x10,0x0F):
        bc=random.choice([2*c1, 2*c1, (2*c1+1)&0xFF, (c1+7)//8])
        data=bytes(random.getrandbits(8) for _ in range(bc))
        add(bytes([sl,fc])+struct.pack('>HHB',a,c1,bc)+data)
    elif fc==0x16:
        add(bytes([sl,fc])+struct.pack('>HHH',a,random.getrandbits(16),random.getrandbits(16)))
    else:
        v=random.choice([c1,0xFF00,0x0000,random.getrandbits(16)])
        add(bytes([sl,fc])+struct.pack('>HH',a,v))
open('frames2.txt','w').write('\n'.join(F)+'\n')
print(len(F),"vollstaendige Telegramme")
