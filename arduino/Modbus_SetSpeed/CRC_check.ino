#define SSerialTxControl 3   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

int byteReceived;
int byteSend;

#define KEY_NOTHING 0x0
#define KEY_Stop 0x1
#define KEY_Start 0x2
#define KEY_Change 0x3
#define KEY_HighSpeed 0x4
#define KEY_MiddleSpeed 0x5
#define KEY_LowSpeed 0x6

#define LEDPin 13

#define MODBUS_ID 0x01
#define MODBUS_FC06_BYTE 0x06
#define MODBUS_ADDR_SPEED 0x20

unsigned int High_frequency = 5000;
unsigned int Middle_frequency = 3000;
unsigned int Low_frequency = 1000;

unsigned char Motion_val;
unsigned char BL_Receive_val;
unsigned char BL_Buffer[4];
unsigned char BL_len;

unsigned char BT_commend;

byte SetSpeedCommend[] = {0x01, 0x06, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00};

byte StopCommend[] = {0x01, 0x06, 0x20, 0x00, 0x00, 0x01, 0x43, 0xCA};
//modbus buffer
byte Modbus_data[5];
byte byteRead;
unsigned int Modbus_len;


void setup()
{
  // USB debig port 0
  Serial.begin(9600);
  // bluetooth port 1
  Serial1.begin(9600);
  // modbus port 2
  Serial2.begin(9600, SERIAL_8N2);
  // = SlaveNode.begin(9600, SERIAL_8N2);
  pinMode(SSerialTxControl, OUTPUT);
  pinMode(LEDPin, OUTPUT);
  digitalWrite(LEDPin, LOW);

}


void loop() {
  // SetSpeedCommend[4] = BL_Buffer[3];
  // SetSpeedCommend[5] = BL_Buffer[4];
  UInt16 CRC_check = 0;
  CRC_check = ModRTU_CRC(StopCommend, 5);
  Serial.print("CRC_check :");
  Serial.print(CRC_check);
  Serial.print(", HEX :");
  Serial.println(CRC_check, HEX);
  delay(1000);
}



UInt16 ModRTU_CRC(byte[] buf, int len)
{
  UInt16 crc = 0xFFFF;

  for (int pos = 0; pos < len; pos++) {
    crc ^= (UInt16)buf[pos];          // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;
}