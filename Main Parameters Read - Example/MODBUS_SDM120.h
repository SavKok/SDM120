/**
  ******************************************************************************
  * @file    SDM120.h
  * @author  Savvas Kokkinidis - sabbaskok@hotmail.com
  * @version V1.0
  * @date    23/October/2022
  * @brief   This file contains the definitions related to the
  *          the MODBUS SDM120 implementation
  ******************************************************************************
*/

#ifndef __MODBUS_SDM120_H
#define __MODBUS_SDM120_H

#include <Arduino.h>
#include <stdint.h>

#define DEFAULT_PROTOCOL    SERIAL_8N1
#define DEFAULT_BAUD_RATE         2400
#define DEFAULT_NODE_ID              1
#define DEFAULT_RS485_DE_PIN        11   
#define DEFAULT_RS485_RE_PIN        12 
#define TOTAL_READ_COMMANDS          3
#define READ_CMD_TOTAL_SIZE          8
#define READ_RSP_TOTAL_SIZE          9
#define READ_CMD_SIZE                5 /* Do not count the 2 bytes for CRC and the node id*/
#define RESPONSE_TIMEOUT_MS        200

#define SDM120_DEBUG                 

typedef enum FunctionCode: byte {
  FUNCTION_READ_HOLDING_REGISTERS   = 0x03,
  FUNCTION_READ_INPUT_REGISTERS     = 0x04,
  FUNCTION_WRITE_HOLDING_REGISTERS  = 0x10
};

typedef enum ResponseCode: byte {
  RESPONSE_OK,
  RESPONSE_NOK,
  RESPONSE_TIMEOUT,
  RESPONSE_INLVAID_FORMAT,
  RESPONSE_INLVAID_CRC
};

typedef enum ReadParameters: byte {
  VOLTAGE,
  CURRENT,
  FREQUENCY
};

typedef enum CrcCheckResult: byte {
  CRC_OK,
  CRC_INVALID
};

typedef enum ModbusProtocolAddresses: byte {
  READ_VOLTAGE_HIGH     = 0x00,
  READ_VOLTAGE_LOW      = 0x00,

  READ_CURRENT_HIGH     = 0x00,
  READ_CURRENT_LOW      = 0x06,

  READ_FREQUENCY_HIGH   = 0x00,
  READ_FREQUENCY_LOW    = 0x46
};

typedef union {
  struct {        
    uint8_t Function;
    uint8_t StaringAddressHigh;
    uint8_t StaringAddressLow;
    uint8_t NumberOfPointsHigh;
    uint8_t NumberOfPointsLow;
  } ReadStruct;
  uint8_t bytes[READ_CMD_SIZE];
} SDM120_READ_UNION;

typedef union {
  uint8_t bytes[4];
  float value;
} FloatUnion;

const SDM120_READ_UNION ReadCommands[]  = {{FUNCTION_READ_INPUT_REGISTERS, READ_VOLTAGE_HIGH,   READ_VOLTAGE_LOW,   0x00, 0x02},
                                           {FUNCTION_READ_INPUT_REGISTERS, READ_CURRENT_HIGH,   READ_CURRENT_LOW,   0x00, 0x02},
                                           {FUNCTION_READ_INPUT_REGISTERS, READ_FREQUENCY_HIGH, READ_FREQUENCY_LOW, 0x00, 0x02}};

class SDM120
{
  public:

    SDM120();
    SDM120(HardwareSerial& serial, uint32_t baudRate, uint8_t protocol, uint8_t dePin, uint8_t rePin);
    void begin(const byte _nodeId);   
    ResponseCode SDM120::getValue(ReadParameters parameter, float* value);

  private:
    void serialFlush(void);
    void setRead(void);
    void setWrite(void);
    float SDM120::bytesToFloat(uint8_t* regData);
    bool sendReadCMD(uint8_t index);
    ResponseCode SDM120::getReadRSP(uint8_t* response,ReadParameters parameter);
    uint16_t calculateCrc(uint8_t* data, uint8_t size);
    CrcCheckResult SDM120::checkCrc(uint8_t* data, uint8_t size, uint8_t crcLow, uint8_t crcHigh);

    HardwareSerial& _SDM120Serial;

    uint8_t _nodeId;
    uint8_t _highAddrByte;
    uint8_t _lowAddrByte;

    uint32_t _baudRate;
    uint32_t _protocol;
    uint8_t  _dePin;
    uint8_t  _rePin;     
};

#endif /* __MODBUS_SDM120_H */