#include "MODBUS_SDM120.h"

#define BAUD_RATE                   2400
#define ID                             1
#define PROTOCOL              SERIAL_8N1
#define RS485_CONTROL_DE              3
#define RS485_CONTROL_RE              4
#define PERIOD_READING             1000

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
  static float Value;
  String info = "";
  String unit = "";

  if(millis() > (time + PERIOD_READING)) {
    switch (packetToSend)
    {
      case VOLTAGE:
        info = "  Voltage: ";
        unit = "V";
        SDM120_Device.getValue(VOLTAGE, &Value);
        packetToSend = CURRENT;
        break;
      case CURRENT:
        info = "  Current: ";
        unit = "A";
        SDM120_Device.getValue(CURRENT, &Value);
        packetToSend = FREQUENCY;
        break;
      case FREQUENCY:
        info = "  Frequency: ";
        unit = "Hz";
        SDM120_Device.getValue(FREQUENCY, &Value);
        packetToSend = VOLTAGE;
        break;
      default:
        packetToSend = VOLTAGE;
        break;
    }

    time = millis();

    Serial.print(info);
    Serial.print(Value, 2);
    Serial.println(unit);    
  }
}
