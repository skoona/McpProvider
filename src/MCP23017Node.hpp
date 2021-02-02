/**
 * Homie Node for MCP23017 I2C port Expander
 * 
 * 
 */

#pragma once

#include <Homie.hpp>
#include <cppQueue.h>
#include <brzo_i2c.h>

#define I2C_KHZ 400

class MCP23017Node : public HomieNode {

public:
  MCP23017Node(const uint8_t sdaPin, const uint8_t sclPin, const uint8_t isrPin, const uint8_t devAddr, const char *id, const char *name, const char *nType);
  void setInversionPolarityAB(uint8_t valueA, uint8_t valueB) { _ipolASetting = valueA; _ipolBSetting = valueB; }

protected:
  void setup() override;
  void loop() override;
  
private:
  uint8_t _sdaPin;
  uint8_t _sclPin;
  uint8_t _isrPin; 
  uint8_t _devAddr;

  const char *cCaption = "• MCP23017 WaveShare Module:";
  const char* cIndent  = "  ◦ ";

  const int  _numPins = 16;
  const char *cPropertyBase = "pin";
  const char *cPropertyBaseName = "Pin ";
  const char *cPropertyBaseDataType = "enum";
  const char *cPropertyBaseFormat = "OPEN,CLOSED";

  volatile bool _runState = false;
  volatile byte _isrTrigger = LOW;
  volatile unsigned long _isrTriggeredAt = 0L;

  volatile bool interruptDataLoss = false;
  volatile unsigned long events = 0;
  volatile bool clearInterruptCycle = false;
  char bufferA[8];
  char bufferB[8];
  char cMesg1[24];
  char cMesg2[24];

  typedef struct __attribute__((packed)) _McpIState
  {
    uint8_t intfA;
    uint8_t intfB;
    uint8_t intcapA;
    uint8_t intcapB;
} McpIState, *PMcpIState;

uint8_t _ipolASetting = 0xff;
uint8_t _ipolBSetting = 0xff;

cppQueue *mcpQueue;
McpIState mcp;

void interruptHandler();
byte mcpClearInterrupts();
byte mcpInit();

};
