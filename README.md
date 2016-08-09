# Inverter_app
AC motor + BLE andoird app + arduino

# BT communication protocol
header1, header 2, commend, reserve, reserve
0x41, 0x42, 0x55, 0x??, 0x??

* Commend
High_frequency 5000 = 0x55
Middle_frequency 3000 = 0x56 
Low_frequency 1000 = speed 
Stop = 0x50 
Start = 0x53
Change = 0x52
Nothing = 0x00
Specific speed = 0x58

* Specific speed commend : 0x41, 0x42, 0x58, HIGHBYTE, LOWBYTE
packet 0x41,0x42,0x58,0x10,0x10 -> speed 0x1010 = 4112
