#include <SPI.h>
#include "DxlMaster.h"

//Control Table Address Switch
#define USW_DATA 27             //uint8_t 0..1 return status
DynamixelDevice device(23);

#define CAN_2515
#define MAX_DATA_SIZE 7
#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)
// For Wio Terminal w/ MCP2518FD RPi Hat：
// Channel 0 SPI_CS Pin: BCM 8
// Channel 1 SPI_CS Pin: BCM 7
// Interupt Pin: BCM25
const int SPI_CS_PIN = BCM8;
const int CAN_INT_PIN = BCM25;
#else

// For Arduino MCP2515 Hat:
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;//9
const int CAN_INT_PIN = 2;
#endif

#ifdef CAN_2518FD
#include "mcp2518fd_can.h"
mcp2518fd CAN(SPI_CS_PIN); // Set CS pin
#endif

#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#endif


void setup() {
  // put your setup code here, to run once:
  
    Serial.begin(1000000);

  while (CAN_OK != CAN.begin(CAN_1000KBPS, MCP_8MHz)) { // init can bus : baudrate = 500k
    //        SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
    //Serial.println("CAN init fail, retry...");
    delay(10);
  }
    
    DxlMaster.begin(57600);
    while (device.ping() != DYN_STATUS_OK);
}

uint8_t stmp[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
char cdata[MAX_DATA_SIZE] = {0};
uint32_t id;
uint8_t type; // bit0: ext, bit1: rtr
uint8_t len;

void loop() {
  
  //read switch
  uint8_t data_sw;
  device.read(USW_DATA, data_sw);
  
  char data_sw_con;
  if(data_sw == 0)
  {
    data_sw_con = 1;
  }
  else
  {
    data_sw_con = 8;
  }

  delay(20);

// read from serial and send to can
  int lenght = Serial.available();
  if (lenght > 0) {
    for (int i = 0; i < lenght; i++) {
 
      uint8_t data = Serial.read();
      
      if (data == 0xAC) {
        uint8_t a = Serial.read();
        uint8_t b = Serial.read();
        uint8_t c = Serial.read();
        uint8_t d = Serial.read();

        id = a | (uint32_t(b) << 8) | (uint32_t(c) << 16) | (uint32_t(d) << 24);

        uint8_t msglen = Serial.read();
        for (int i = 0; i < 3; i++) {
          Serial.read();
        }
        for (uint8_t e = 0; e < msglen; e++) {
          stmp[e] = Serial.read();
        }
        break;
      }
    }
    
    CAN.sendMsgBuf(id, 0, 8, stmp);
  }
  delay(10);
    
//send CAN message
  
  unsigned char len;
  if(CAN_MSGAVAIL == CAN.checkReceive())
    {
      CAN.readMsgBuf(&len, cdata);

      cdata[6] = data_sw_con;
      for (int i = 0; i < 7; i++) {
        Serial.write(cdata[i]);
      }
  delay(10);
  }
}
  
    
