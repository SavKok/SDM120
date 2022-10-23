#include "MODBUS_SDM120.h"

#define BAUD_RATE                   2400
#define ID                             1
#define PROTOCOL              SERIAL_8N1
#define RS485_CONTROL_DE              3
#define RS485_CONTROL_RE              4

static uint8_t packetToSend = VOLTAGE;
static unsigned long time;
int incomingByte = 0; // for incoming serial data

SDM120 SDM120_Device(Serial1, BAUD_RATE, PROTOCOL, RS485_CONTROL_DE, RS485_CONTROL_RE);

void setup() {
  Serial.begin(2400);
  SDM120_Device.begin(ID);
  time = millis();
}

void loop() {
  if(millis() > (time + 2000U)) {
    switch (packetToSend)
    {
      case VOLTAGE:
        if(SDM120_Device.sendReadCMD(VOLTAGE) == true) {
          if(SDM120_Device.getReadRSP() == RESPONSE_OK)
          {
            packetToSend = VOLTAGE; /* debug stuck here */
          }          
        }
        break;
      case CURRENT:
        SDM120_Device.sendReadCMD(CURRENT);
        packetToSend = FREQUENCY;
        break;
      case FREQUENCY:
        SDM120_Device.sendReadCMD(FREQUENCY);
        packetToSend = VOLTAGE;
        break;
      default:
        packetToSend = VOLTAGE;
        break;
    }
    time = millis();
  }

  // if (Serial.available() > 0) {
  //   // read the incoming byte:
  //   incomingByte = Serial.read();

  //   Serial.print(incomingByte,HEX);
  // }

}
