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

unsigned int High_frequency = 5000;
unsigned int Middle_frequency = 3000;
unsigned int Low_frequency = 1000;

unsigned char Motion_val;
unsigned char BL_Receive_val;
unsigned char BL_Buffer[4];
unsigned char BL_len;

unsigned char BT_commend;

// commend for converter
byte StopCommend[] = {0x01,0x06,0x20,0x00,0x00,0x01,0x43,0xCA};
byte StartCommend[] = {0x01,0x06,0x20,0x00,0x00,0x02,0x03,0xCB};
byte ChangeCommend[] = {0x01,0x06,0x20,0x00,0x00,0x30,0x82,0x1E};

byte HighSpeed[] = {0x01,0x06,0x20,0x01,0x13,0x88,0xDE,0x9C};
byte MiddleSpeed[] = {0x01,0x06,0x20,0x01,0x0B,0xB8,0xD4,0x88};
byte LowSpeed[] = {0x01,0x06,0x20,0x01,0x03,0xE8,0xD3,0x74};

byte ReadStatus[] = {0x01,0x03,0x21,0x00,0x00,0x01,0x8E,0x36};

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
  // main 1.read BT commend 2.Write commend to Delta VFD 3.Read error code from Delta VFD
  Motion_val = KEY_NOTHING;
  UART_Bluetooth();
  VFD_Control(Motion_val);
  VFD_Monitor();

}

void VFD_Control(unsigned char Control_input) {
  // address:2000H

  digitalWrite(SSerialTxControl, RS485Transmit);  // Enable RS485 Transmit   
  
  if (Control_input == KEY_Stop) {
    for (int i = 0; i < 8 ; i++){
      Serial2.write(StopCommend[i]);          // Send byte to Remote Arduino
    }
  }
  else if (Control_input == KEY_Start) {
    for (int i = 0; i < 8 ; i++){
      Serial2.write(StartCommend[i]);          // Send byte to Remote Arduino
    }
    
  }
  else if (Control_input == KEY_Change) {
    for (int i = 0; i < 8 ; i++){
      Serial2.write(ChangeCommend[i]);          // Send byte to Remote Arduino
    }
  }
  else if (Control_input == KEY_HighSpeed) {
    for (int i = 0; i < 8 ; i++){
      Serial2.write(HighSpeed[i]);          // Send byte to Remote Arduino
    }
  }
  else if (Control_input == KEY_MiddleSpeed) {
    for (int i = 0; i < 8 ; i++){
      Serial2.write(MiddleSpeed[i]);          // Send byte to Remote Arduino
    }
  }
  else if (Control_input == KEY_LowSpeed) {
    for (int i = 0; i < 8 ; i++){
      Serial2.write(LowSpeed[i]);          // Send byte to Remote Arduino
    }
  }
  delay(50);
}

void VFD_Monitor() {
  // Read Delta Converter modbus address 2100
  
  // clear the old buffer
//  digitalWrite(SSerialTxControl, RS485Receive);  // Enable  RS485Receive
//  delay(20);
//  Serial2.flush();

  // read 2100H status
  digitalWrite(SSerialTxControl, RS485Transmit);  // Enable  RS485Receive
  for (int i = 0; i < 8 ; i++){
      Serial2.write(ReadStatus[i]);          // Send byte to Remote Arduino
  }
  delay(10);

  // get response
  digitalWrite(SSerialTxControl, RS485Receive);  // Enable  RS485Receive

  //initilize the received_buffer
  for (int i = 0; i < 5; i++) {
    Modbus_data[i] = 0x0;
  }
  
  //get modbus response from converter
  while(Serial2.available())
  {
    byteRead = Serial2.read();
    // the first index
    if (byteRead == 0x01) {
      Modbus_len = 0;
      Modbus_data[Modbus_len++] = byteRead;
    }
    else if (Modbus_len < 6) {
      Modbus_data[Modbus_len++] = byteRead;
    }
  }
  
  //match the correct system status
  if (Modbus_data[1] == 0x03 && Modbus_data[2] == 0x02 && Modbus_data[3] == 0x0 && Modbus_data[4] == 0x0 ){
      digitalWrite(LEDPin, HIGH);
      Serial.println("R");
      // send to BT
      Serial1.print("R");
  }
  //incorrect system status
  else {
      digitalWrite(LEDPin, LOW);
      Serial.println("ERR");
      // send to BT
      Serial1.print("E");
  }
  
  delay(50);
}

void UART_Bluetooth() {
  // initial BL_Buffer = 0x0
  for (int i = 0; i < 5; i++) {
    BL_Buffer[i] = 0x0;
  }
  delay(200);


  while (Serial1.available())
  {
    BL_Receive_val = Serial1.read();

    // if get 0x41 -> start byte, receive data to BL_Buffer
    if (BL_Receive_val == 0x41) {
      BL_len = 0;
      BL_Buffer[BL_len++] = BL_Receive_val;
    }
    else if (BL_len < 5) {
      BL_Buffer[BL_len++] = BL_Receive_val;
    }
  }
  // get the BT packet :
  // 55 ,56,57 = speed , 50 = stop ,53 = start, 52= change, 00 = nothing
  // Return the Key definition.
  if (BL_Buffer[0] == 0x41 && BL_Buffer[1] == 0x42 && BL_Buffer[2] == 0x55) {
    Motion_val = KEY_HighSpeed;
  }
  else if (BL_Buffer[0] == 0x41 && BL_Buffer[1] == 0x42 && BL_Buffer[2] == 0x56) {
    Motion_val = KEY_MiddleSpeed;
  }
  else if (BL_Buffer[0] == 0x41 && BL_Buffer[1] == 0x42 && BL_Buffer[2] == 0x57) {
    Motion_val = KEY_LowSpeed;
  }
  else if (BL_Buffer[0] == 0x41 && BL_Buffer[1] == 0x42 && BL_Buffer[2] == 0x50) {
    Motion_val = KEY_Stop;
  }
  else if (BL_Buffer[0] == 0x41 && BL_Buffer[1] == 0x42 && BL_Buffer[2] == 0x53) {
    Motion_val = KEY_Start;
  }
  else if (BL_Buffer[0] == 0x41 && BL_Buffer[1] == 0x42 && BL_Buffer[2] == 0x52) {
    Motion_val = KEY_Change;
  }
  else if (BL_Buffer[0] == 0x41 && BL_Buffer[1] == 0x42 && BL_Buffer[2] == 0x00) {
    Motion_val = KEY_NOTHING;
  }


}


