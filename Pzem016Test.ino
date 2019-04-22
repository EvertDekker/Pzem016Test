/*
  An Arduino Sketch for reading data from a PZEM-014 or PZEM-016, tested with ESP32 DEVKit 1, Arduino 1.8.5
  EvertDekker.com 2018, based on the example from http://solar4living.com/pzem-arduino-modbus.htm

  If you want to use slaveid function to change the slaveid on the fly, you need to modify the ModbusMaster library (Or get the copy from my website)
  In ModbusMaster.h add at line 78
    void slaveid(uint8_t);
  In ModbusMaster.cpp add at line 75
    void ModbusMaster::slaveid(uint8_t slave)
     {
      _u8MBSlave = slave;
     }
*/
/* If you are using other then uart0 on the ESP32, Comment out in esp32-hal-uart.c the follwing line:
  //uart->dev->conf0.txfifo_rst = 1;
  //uart->dev->conf0.txfifo_rst = 0;
  //uart->dev->conf0.rxfifo_rst = 1;
  //uart->dev->conf0.rxfifo_rst = 0;
  Source: https://github.com/4-20ma/ModbusMaster/issues/93
*/

#include <ModbusMaster.h>

HardwareSerial Pzemserial(2);

#define RXD2 16 //Gpio pins Serial2
#define TXD2 17

#define MAX485_DE      19  // We're using a MAX485-compatible RS485 Transceiver. The Data Enable and Receiver Enable pins are hooked up as follows:
#define MAX485_RE_NEG  18

ModbusMaster node;
static uint8_t pzemSlaveAddr = 0x01;

void setup() {
  Pzemserial.begin(9600, SERIAL_8N1, RXD2, TXD2);  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial.begin(9600);
  node.begin(pzemSlaveAddr, Pzemserial);  //Start the Modbusmaster

  pinMode(MAX485_RE_NEG, OUTPUT);  // Setting up the RS485 transceivers
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);  // Init in receive mode
  digitalWrite(MAX485_DE, 0);

  node.preTransmission(preTransmission);  // Callbacks allow us to configure the RS485 transceiver correctly
  node.postTransmission(postTransmission);

  //changeAddress(0x01, 0x02);
  /* By Uncomment the function in the above line you can change the slave address from one of the nodes, only need to be done ones. Preverable do this only with 1 slave in the network.
     changeAddress(OldAddress, Newaddress)
     If you f*ck it up or don't know the new address anymore, you can use the broadcast address 0XF8 as OldAddress to change the slave address. Use this with one slave ONLY in the network.
  */

  //resetEnergy(0x01);
  /* By Uncomment the function in the above line you can reset the energy counter (Wh) back to zero from one of the slaves.
  */

  delay(1000);
}

/*
  RegAddr Description                 Resolution
  0x0000  Voltage value               1LSB correspond to 0.1V
  0x0001  Current value low 16 bits   1LSB correspond to 0.001A
  0x0002  Current value high 16 bits
  0x0003  Power value low 16 bits     1LSB correspond to 0.1W
  0x0004  Power value high 16 bits
  0x0005  Energy value low 16 bits    1LSB correspond to 1Wh
  0x0006  Energy value high 16 bits
  0x0007  Frequency value             1LSB correspond to 0.1Hz
  0x0008  Power factor value          1LSB correspond to 0.01
  0x0009  Alarm status  0xFFFF is alarm，0x0000is not alarm
*/

void loop() {
  uint8_t result;

  for (pzemSlaveAddr = 1; pzemSlaveAddr < 3; pzemSlaveAddr++) {  // Loop all the Pzem sensors
    node.slaveid(pzemSlaveAddr);          //Switch to another slave address. NOTE: You can only use this function is you have modified the ModbusMaster library (Or get the copy from my website)
    Serial.print("Pzem Slave ");
    Serial.print(pzemSlaveAddr);
    Serial.print(": ");
   
    result = node.readInputRegisters(0x0000, 9); //read the 9 registers of the PZEM-014 / 016
    if (result == node.ku8MBSuccess)
    {
      uint32_t tempdouble = 0x00000000;

      float voltage = node.getResponseBuffer(0x0000) / 10.0;  //get the 16bit value for the voltage, divide it by 10 and cast in the float variable 

      tempdouble =  (node.getResponseBuffer(0x0002) << 16) + node.getResponseBuffer(0x0001);  // Get the 2 16bits registers and combine them to an unsigned 32bit
      float current = tempdouble / 1000.00;   // Divide the unsigned 32bit by 1000 and put in the current float variable 

      tempdouble =  (node.getResponseBuffer(0x0004) << 16) + node.getResponseBuffer(0x0003);
      float power = tempdouble / 10.0;

      tempdouble =  (node.getResponseBuffer(0x0006) << 16) + node.getResponseBuffer(0x0005);
      float energy = tempdouble;

      float hz = node.getResponseBuffer(0x0007) / 10.0;
      float pf = node.getResponseBuffer(0x0008) / 100.00;

      Serial.print(voltage, 1);  // Print Voltage with 1 decimal
      Serial.print("V   ");

      Serial.print(hz, 1);
      Serial.print("Hz   ");

      Serial.print(current, 3);
      Serial.print("A   ");

      Serial.print(power, 1);
      Serial.print("W  ");

      Serial.print(pf, 2);
      Serial.print("pf   ");

      Serial.print(energy, 0);
      Serial.print("Wh  ");
      Serial.println();
      if (pzemSlaveAddr==2){Serial.println(); }
    } else
    {
      Serial.println("Failed to read modbus");
    }
    delay(1000);
  }

}

void preTransmission()  // Put RS485 Transceiver in transmit mode
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
  delay(1);
}

void postTransmission()  // Put RS485 Transceiver back in receive mode (default mode)
{
  delay(3);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void resetEnergy(uint8_t slaveAddr)    //Reset the slave's energy counter
{
  uint16_t u16CRC = 0xFFFF;
  static uint8_t resetCommand = 0x42;
  u16CRC = crc16_update(u16CRC, slaveAddr);
  u16CRC = crc16_update(u16CRC, resetCommand);
  Serial.println("Resetting Energy");
  preTransmission();
  Pzemserial.write(slaveAddr);
  Pzemserial.write(resetCommand);
  Pzemserial.write(lowByte(u16CRC));
  Pzemserial.write(highByte(u16CRC));
  delay(10);
  postTransmission();
  delay(100);
  while (Pzemserial.available()) {         // Prints the response from the Pzem, do something with it if you like
    Serial.print(char(Pzemserial.read()), HEX);
    Serial.print(" ");
  }
}

void changeAddress(uint8_t OldslaveAddr, uint8_t NewslaveAddr)  //Change the slave address of a node
{
  static uint8_t SlaveParameter = 0x06;
  static uint16_t registerAddress = 0x0002; // Register address to be changed
  uint16_t u16CRC = 0xFFFF;
  u16CRC = crc16_update(u16CRC, OldslaveAddr);  // Calculate the crc16 over the 6bytes to be send
  u16CRC = crc16_update(u16CRC, SlaveParameter);
  u16CRC = crc16_update(u16CRC, highByte(registerAddress));
  u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
  u16CRC = crc16_update(u16CRC, highByte(NewslaveAddr));
  u16CRC = crc16_update(u16CRC, lowByte(NewslaveAddr));

  Serial.println("Change Slave Address");
  preTransmission();
  Pzemserial.write(OldslaveAddr);
  Pzemserial.write(SlaveParameter);
  Pzemserial.write(highByte(registerAddress));
  Pzemserial.write(lowByte(registerAddress));
  Pzemserial.write(highByte(NewslaveAddr));
  Pzemserial.write(lowByte(NewslaveAddr));
  Pzemserial.write(lowByte(u16CRC));
  Pzemserial.write(highByte(u16CRC));
  delay(10);
  postTransmission();
  delay(100);
  while (Pzemserial.available()) {   // Prints the response from the Pzem, do something with it if you like
    Serial.print(char(Pzemserial.read()), HEX);
    Serial.print(" ");
  }
}
