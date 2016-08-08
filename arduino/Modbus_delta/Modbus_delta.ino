/////// 1.modbus ////////
/* 
modbus function code http://www.simplymodbus.ca/
Write functiuon
 writeSingleRegister Modbus function 0x06 Write Single Register. # 4x
 
Read functiuon
 readHoldingRegisters Modbus function 0x03 Read Holding Register. # 4x
*/
#include <ModbusMaster.h>

// instantiate ModbusMaster object as slave ID 1
uint8_t MobusComPort = 0;
uint8_t SlaveID = 1;
ModbusMaster SlaveNode(MobusComPort, SlaveID);

/////// 2.Bluetooth ///////
#include <SoftwareSerial.h>
#define RxD 2
#define TxD 3
#define LED 13
SoftwareSerial blueToothSerial(RxD,TxD); 

#define KEY_NOTHING 0x0
#define KEY_Stop 0x1
#define KEY_Forward 0x2
#define KEY_Backward 0x3
#define KEY_Accelerate 0x4

unsigned char Motion_val;
unsigned char BL_Receive_val;
unsigned char BL_Buffer[4];
unsigned char len;

unsigned char result;
unsigned char BT_commend;

void setup()
{
  // initialize Modbus communication baud rate
  SlaveNode.begin(9600);
  // 8N2 , it look ok now, but it need to repair.
  // UCSR0C=0x0E;
  //Serial.begin(9600);
  blueToothSerial.begin(9600);
}


void loop(){
  // main 1.read BT commend 2.Write commend to Delta VFD 3.Read error code from Delta VFD
  UART_Bluetooth();
  result = VFD_Control(Motion_val);
  
//  VFD_Monitor();
}

uint8_t VFD_Control(unsigned char Control_input){
  // address:2000H bit0-5
  // 0x1: stop      000001 = 1
  // 0x2: forward   010010 = 18
  // 0x3. backward  100010 = 34
  // 0x4. change    110010 = 50
  static uint32_t i;
  uint8_t W_result;
  i++;
  
  // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
  SlaveNode.setTransmitBuffer(0, lowWord(i)); 
  // set word 1 of TX buffer to most-significant word of counter (bits 31..16)
  SlaveNode.setTransmitBuffer(1, highWord(i));
  
  result = SlaveNode.writeSingleRegister(0, 4);
  if(Control_input == KEY_Stop){
    W_result = SlaveNode.writeSingleRegister(2000, 1);
  }
  else if(Control_input == KEY_Forward){
    W_result = SlaveNode.writeSingleRegister(2000, 18);
  }
  else if(Control_input == KEY_Backward){
    W_result = SlaveNode.writeSingleRegister(2000, 34);
  }
  else if(Control_input == KEY_Accelerate){
    W_result = SlaveNode.writeSingleRegister(2000, 50);
  }
  
  
  return W_result;
}

void VFD_Monitor(){
  // Read Delta Converter modbus address 2100
  uint16_t data[6];
  uint8_t R_result;
  
  R_result = SlaveNode.readHoldingRegisters(2100, 1); 
  
  if (R_result == SlaveNode.ku8MBSuccess)
  {
    Serial.print("Monitor: ");
    for (int j = 0; j < 2; j++)
    {
      data[j] = SlaveNode.getResponseBuffer(j);
      // slave: write data buffer to (data[j]) 16-bit registers starting at register 4x00003
      Serial.print(data[j]);  
      Serial.print(" "); 
    }
    Serial.println();
  }
}


void UART_Bluetooth(){
  // initial BL_Buffer = 0x0
  for(int i=0;i<5;i++){
   BL_Buffer[i]=0x0;
  }
  delay(100);
  // Receive the BL data to BL_Buffer 

  while(blueToothSerial.available())            
  {                                                
     BL_Receive_val = blueToothSerial.read();
     // if get 0xAA -> start byte, receive data to BL_Buffer
     if(BL_Receive_val == 0xAA){
       len = 0;
       BL_Buffer[len++] = BL_Receive_val;
     }
     else if(len < 5){
       BL_Buffer[len++] = BL_Receive_val;
     }   
  }   
  // get the BT packet : 
  // F1 = Accelerate ,F2 = stop , F4 = Forward , F8 = Backward
  // Return the Key definition.
  if(BL_Buffer[0]==0xAA && BL_Buffer[1]==0xBB && BL_Buffer[3]==0xF1){
    Serial.print("Control Accelerate \n"); 
    Motion_val = KEY_Accelerate;
  }
  else if(BL_Buffer[0]==0xAA && BL_Buffer[1]==0xBB && BL_Buffer[3]==0xF2){
    Serial.print("Control Stop \n");
    Motion_val = KEY_Stop;
  }
  else if(BL_Buffer[0]==0xAA && BL_Buffer[1]==0xBB && BL_Buffer[3]==0xF4){
    Serial.print("Control Forward \n"); 
    Motion_val = KEY_Forward;
  }
  else if(BL_Buffer[0]==0xAA && BL_Buffer[1]==0xBB && BL_Buffer[3]==0xF8){
    Serial.print("Control Backward \n"); 
    Motion_val = KEY_Backward;
  }
  else if (BL_Buffer[0]==0xAA && BL_Buffer[1]==0xBB && BL_Buffer[3]==0x00){
    Serial.print("Nothing \n"); 
    Motion_val = KEY_NOTHING;
  }
}

