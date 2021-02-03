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

// registers
#define MCP23017_IODIRA 0x00   //!< I/O direction register A
#define MCP23017_IPOLA 0x02    //!< Input polarity port register A
#define MCP23017_GPINTENA 0x04 //!< Interrupt-on-change pins A
#define MCP23017_DEFVALA 0x06  //!< Default value register A
#define MCP23017_INTCONA 0x08  //!< Interrupt-on-change control register A
#define MCP23017_IOCONA 0x0A   //!< I/O expander configuration register A
#define MCP23017_GPPUA 0x0C    //!< GPIO pull-up resistor register A
#define MCP23017_INTFA 0x0E    //!< Interrupt flag register A
#define MCP23017_INTCAPA 0x10  //!< Interrupt captured value for port register A
#define MCP23017_GPIOA 0x12    //!< General purpose I/O port register A
#define MCP23017_OLATA 0x14    //!< Output latch register 0 A

#define MCP23017_IODIRB 0x01   //!< I/O direction register B
#define MCP23017_IPOLB 0x03    //!< Input polarity port register B
#define MCP23017_GPINTENB 0x05 //!< Interrupt-on-change pins B
#define MCP23017_DEFVALB 0x07  //!< Default value register B
#define MCP23017_INTCONB 0x09  //!< Interrupt-on-change control register B
#define MCP23017_IOCONB 0x0B   //!< I/O expander configuration register B
#define MCP23017_GPPUB 0x0D    //!< GPIO pull-up resistor register B
#define MCP23017_INTFB 0x0F    //!< Interrupt flag register B
#define MCP23017_INTCAPB 0x11  //!< Interrupt captured value for port register B
#define MCP23017_GPIOB 0x13    //!< General purpose I/O port register B
#define MCP23017_OLATB 0x15    //!< Output latch register 0 B

#define MCP23017_INT_ERR 255 //!< Interrupt error
class MCP23017Node : public HomieNode {

public:
  MCP23017Node(const uint8_t sdaPin, const uint8_t sclPin, const uint8_t isrPin, const uint8_t devAddr, const char *id, const char *name, const char *nType, const int measurementInterval = MEASUREMENT_INTERVAL);
  void setInversionPolarityAB(uint8_t valueA, uint8_t valueB) { _ipolASetting = valueA; _ipolBSetting = valueB; }
  void setMeasurementInterval(unsigned long interval) { _measurementInterval = interval; }

protected:
  void setup() override;
  void loop() override;
  
private:
  uint8_t _sdaPin;
  uint8_t _sclPin;
  uint8_t _isrPin; 
  uint8_t _devAddr;

  // suggested rate is 1/60Hz (1m)
  static const int MIN_INTERVAL = 60; // in seconds
  static const int MEASUREMENT_INTERVAL = 300;
  unsigned long _measurementInterval;
  unsigned long _lastMeasurement;

  const char *cCaption = "• MCP23017 WaveShare Module:";
  const char* cIndent  = "  ◦ ";

  const unsigned long _InterruptBounce = 250;

  const int  _numPins = 16;
  const char *cPropertyBase = "pin";
  const char *cPropertyBaseName = "Pin ";
  const char *cPropertyBaseDataType = "enum";
  const char *cPropertyBaseFormat = "OPEN,CLOSED";

  volatile byte _isrTrigger = LOW;
  volatile unsigned long _isrTriggeredAt = 0L;
  volatile unsigned long _isrLastTriggeredAt = 0L;

  volatile bool interruptDataLoss = false;
  volatile unsigned long events = 0;
  char cProperty[16][10];       // property
  char cPropertyName[16][10];   // property Title

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
  void handleCurrentState(McpIState *mcp, bool statusOverride);
  byte ICACHE_RAM_ATTR readState();
};
