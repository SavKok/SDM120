/*
  ******************************************************************************
  * @file    MODBUS_SDM120.cpp
  * @author  Savvas Kokkinidis - sabbaskok@hotmail.com
  * @version V1.0
  * @date    23/October/2022
  * @brief   This file contains the functions related to the
  *          the MODBUS SDM120 implementation
  ******************************************************************************
*/

#include "MODBUS_SDM120.h"


SDM120::SDM120(): _SDM120Serial(Serial)
{
  _baudRate = DEFAULT_BAUD_RATE;
  _protocol = DEFAULT_PROTOCOL;
  _dePin    = DEFAULT_RS485_DE_PIN;
  _rePin    = DEFAULT_RS485_RE_PIN;
}

SDM120::SDM120(HardwareSerial& serial, uint32_t baudRate, uint8_t protocol, uint8_t dePin, uint8_t rePin): _SDM120Serial(serial)
{
  _baudRate = baudRate;
  _protocol = protocol;
  _dePin    = dePin;
  _rePin    = rePin;
}

void SDM120::begin(const byte nodeID)
{
    _nodeId = nodeID;
    pinMode(_rePin, OUTPUT);
    pinMode(_dePin, OUTPUT);
    _SDM120Serial.begin(_baudRate, _protocol);
    setRead();                                  /* Start at READ mode */
} 

bool SDM120::sendReadCMD(uint8_t index)
{
  uint8_t result = false;
  uint8_t tmpReadCMD[READ_CMD_TOTAL_SIZE] = {0};

  if(index < TOTAL_READ_COMMANDS) {
      tmpReadCMD[0] = _nodeId;
      memcpy(tmpReadCMD + 1, ReadCommands[index].bytes, READ_CMD_SIZE);

      tmpReadCMD[READ_CMD_TOTAL_SIZE-2] = (uint8_t)  (calculateCrc(tmpReadCMD,6) & 0x00FF);
      tmpReadCMD[READ_CMD_TOTAL_SIZE-1] = (uint8_t) ((calculateCrc(tmpReadCMD,6) & 0xFF00)>>8);

      setWrite();
      delay(1);
      for (uint8_t i = 0; i < READ_CMD_TOTAL_SIZE; i++) {
        _SDM120Serial.write(tmpReadCMD[i]);
      }
      delay(1);
      setRead();
      result = true;
  }

  return result; 
}

ResponseCode SDM120::getReadRSP(uint8_t* response,ReadParameters parameter)
{
  uint8_t  index = 0;  
  uint8_t  result = RESPONSE_TIMEOUT;
  uint32_t time   = millis();
#ifdef SDM120_DEBUG  
  uint8_t  debugResponseV[TOTAL_READ_COMMANDS][READ_RSP_TOTAL_SIZE] = {{0x01, 0x04, 0x04, 0x43, 0x66, 0x33, 0x34, 0x1B, 0x38},
                                                                       {0x01, 0x04, 0x04, 0x3f, 0x99, 0x99, 0x9A, 0xCD, 0x84},
                                                                       {0x01, 0x04, 0x04, 0x42, 0x49, 0x33, 0x33, 0x6A, 0xCF}};
  bool     debugResponseSent = false;
#endif

  serialFlush();

  while(millis() < (time + RESPONSE_TIMEOUT_MS)) {
    if(_SDM120Serial.available() > 0) {
      if(index < READ_RSP_TOTAL_SIZE) { 
        response[index++] = _SDM120Serial.read(); 
      }
    }

#ifdef SDM120_DEBUG
    if((millis() > (time + RESPONSE_TIMEOUT_MS/2)) && (debugResponseSent == false)) {
      Serial.write(&debugResponseV[(uint8_t)parameter][0], READ_RSP_TOTAL_SIZE);
      debugResponseSent = true;
    }
#endif    

    if(index == READ_RSP_TOTAL_SIZE) {      
      if(checkCrc(response, READ_RSP_TOTAL_SIZE-2, response[READ_RSP_TOTAL_SIZE-2], response[READ_RSP_TOTAL_SIZE-1]) == CRC_OK) {
        result = RESPONSE_OK;
      }
      else
      {
        result = RESPONSE_INLVAID_CRC;
      }
      break;
    }
  }

  return result; 
}

ResponseCode SDM120::getValue(ReadParameters parameter, float* value) {
  uint8_t result = RESPONSE_NOK;
  uint8_t  tmpResponse[READ_RSP_TOTAL_SIZE] = {0};

  *value = NAN;

  if(sendReadCMD((uint8_t)parameter) == true) {
    result = getReadRSP(tmpResponse,(uint8_t)parameter);
    if(result == RESPONSE_OK) {
     *value = bytesToFloat(tmpResponse + 3);
    }  
  }

  return result;
}

void SDM120::setRead(void) {
  digitalWrite(_rePin, LOW);
  digitalWrite(_dePin, LOW);
}

void SDM120::setWrite(void) {
  digitalWrite(_rePin, HIGH);
  digitalWrite(_dePin, HIGH);
}

void SDM120::serialFlush(void) {
  while(_SDM120Serial.available() > 0) {
    char t = _SDM120Serial.read();
  }
}

float SDM120::bytesToFloat(uint8_t* regData)
{
    FloatUnion result;

    result.value = NAN;

    result.bytes[0] = regData[3];
    result.bytes[1] = regData[2];
    result.bytes[2] = regData[1];
    result.bytes[3] = regData[0];

    return result.value;
}

uint16_t SDM120::calculateCrc(uint8_t* data, uint8_t size)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < size; i++) {
        crc ^= data[i];
        for (byte j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001; 
            else
                crc = crc >> 1;
        }
    }
    
    return crc;
}

CrcCheckResult SDM120::checkCrc(uint8_t* data, uint8_t size, uint8_t crcLow, uint8_t crcHigh)
{
  uint8_t tmpCrcLow,tmpCrcHigh;
  uint8_t result = CRC_INVALID;

  tmpCrcLow  = (uint8_t)  (calculateCrc(data,size) & 0x00FF);
  tmpCrcHigh = (uint8_t) ((calculateCrc(data,size) & 0xFF00)>>8);

  if((crcLow == tmpCrcLow) && (crcHigh == tmpCrcHigh)) {
    result = CRC_OK;
  }
  
  return result;
}

