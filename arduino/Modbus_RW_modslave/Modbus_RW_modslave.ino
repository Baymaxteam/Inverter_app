/* 
modbus function code http://www.simplymodbus.ca/
Write functiuon
 writeSingleRegister Modbus function 0x06 Write Single Register. # 4x
 writeSingleCoil Modbus function 0x05 Write Single Coil.# 0x
 
Read functiuon
 readCoils Modbus function 0x01 Read Coils. # 0x
 readDiscreteInputs Modbus function 0x02 Read Discrete Inputs. # 1x
 readInputRegisters Modbus function 0x04 Read Input Registers.# 3x
 readHoldingRegisters Modbus function 0x03 Read Holding Register. # 4x
*/
#include <ModbusMaster.h>
//#include <HardwareSerial.h>
// instantiate ModbusMaster object as slave ID 1
uint8_t MobusComPort = 0;
uint8_t SlaveID = 1;
ModbusMaster SlaveNode(MobusComPort, SlaveID);

// defaults to serial port 0 since no port was specified

void setup()
{
  // initialize Modbus communication baud rate
  SlaveNode.begin(19200);
  // 8N2 , it look ok now, but it need to repair.
  // UCSR0C=0x0E;
}


void loop()
{
  static uint32_t i;
  uint8_t j, result;
  uint16_t data[6];
  
  i++;
  
  // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
  SlaveNode.setTransmitBuffer(0, lowWord(i)); 
  // set word 1 of TX buffer to most-significant word of counter (bits 31..16)
  SlaveNode.setTransmitBuffer(1, highWord(i));
  
  // slave: write TX buffer to (02H) 16-bit registers starting at register 4x00000
  // slave: write TX buffer to (04H) 16-bit registers starting at register 4x00001
  result = SlaveNode.writeSingleRegister(2000, 2);
  result = SlaveNode.writeSingleRegister(1, 4);
  result = SlaveNode.writeSingleRegister(7, 8);
  // slave: write TX buffer to (ture) 16-bit registers starting at register 0x00000
  // slave: write TX buffer to (ture) 16-bit registers starting at register 0x00007
  result = SlaveNode.writeSingleCoil(0, 1);
  result = SlaveNode.writeSingleCoil(7, 1);
  
  // slave: read (1) 16-bit registers starting at register 4x00000 to RX buffer
  result = SlaveNode.readInputRegisters(0, 2);
  
  if (result == SlaveNode.ku8MBSuccess)
  {
    for (j = 0; j < 2; j++)
    {
      data[j] = SlaveNode.getResponseBuffer(j);
      // slave: write data buffer to (data[j]) 16-bit registers starting at register 4x00003
      result = SlaveNode.writeSingleRegister(3+j, data[j]);
    }
  }


}

