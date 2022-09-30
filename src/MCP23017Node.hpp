/**
 * Homie Node for MCP23017 I2C port Expander
 * 
 * 
 */

#pragma once

#include <Homie.hpp>
#include <Wire.h>
#include <MCP23017.h>


extern volatile bool gRun;

class MCP23017Node : public HomieNode {

public:
  MCP23017Node(const uint8_t sdaPin, const uint8_t sclPin, const uint8_t isrPin, const uint8_t devAddr, const char *id, const char *name, const char *nType, const int measurementInterval = MEASUREMENT_INTERVAL);
  
  void setInversionPolarityAB(uint8_t valueA, uint8_t valueB) { _ipolASetting = valueA; _ipolBSetting = valueB; Serial.printf(" ✖  setInversionPolarityAB(%d,%d)\n", valueA, valueB); }
  void setMeasurementInterval(unsigned long interval) { _measurementInterval = interval; }
  void sendCurrentStatus() { publishCurrentStatus(true); }

protected:
  void setup() override;
  void onReadyToOperate() override;
  void loop() override;
  
private:
  uint8_t _sdaPin;
  uint8_t _sclPin;
  uint8_t _isrPin; 
  uint8_t _devAddr;
     bool _Once = false;

  // suggested rate is 1/60Hz (1m)
      static const int MIN_INTERVAL = 60; // in seconds
      static const int MEASUREMENT_INTERVAL = 300;
        unsigned long _measurementInterval;
        unsigned long _lastMeasurement;
  const unsigned long _InterruptBounce = 100; // ms

  const char *cCaption = "• MCP23017 WaveShare Module:";
  const char* cIndent  = "  ◦ ";


  const int  _numPins = 16;
  const char *cPropertyBase = "pin";
  const char *cPropertyBaseName = "Pin ";
  const char *cPropertyBaseDataType = "enum";
  const char *cPropertyBaseFormat = "OPEN,CLOSED";

  unsigned long _isrTriggeredAt = 0L;
  unsigned long _isrLastTriggeredAt = 0L;
           bool interruptDataLoss = false;

  unsigned long events = 0;
           char cProperty[16][16];       // property
           char cPropertyName[16][16];   // property Title

  typedef struct __attribute__((packed)) _McpIState
  {
    uint8_t intfA;
    uint8_t intfB;
    uint8_t intcapA;
    uint8_t intcapB;
  } McpIState, *PMcpIState;

  uint8_t _ipolASetting = 0xff;
  uint8_t _ipolBSetting = 0xff;

   MCP23017 mcp;
  McpIState mcpState;

  void handleChangeInterrupts();
  byte begin();
  void publishCurrentStatus(bool readStatus);
};
