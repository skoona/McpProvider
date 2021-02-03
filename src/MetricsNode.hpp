/**
 * Homie Node to overcome OpenHAB's lack of support for device metrics
 * 
 */

#pragma once

#include <Homie.hpp>

#ifdef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP8266 ONLY !)
#error Select ESP8266 board.
#endif


class MetricsNode : public HomieNode {

public:
  MetricsNode(const char *id, const char *name, const char *nType, const int measurementInterval = MEASUREMENT_INTERVAL);
  void setMeasurementInterval(unsigned long interval) { _measurementInterval = interval; }

protected : 
  void setup() override;
  void loop() override;
  
private:

  // suggested rate is 1/60Hz (1m)
  static const int MIN_INTERVAL         = 60;  // in seconds
  static const int MEASUREMENT_INTERVAL = 60;
  unsigned long _measurementInterval;
  unsigned long _lastMeasurement;

  const char* cCaption = "• Device Metrics Node";
  const char* cIndent  = "  ◦ ";

  const char *cPropertySignal = "signal";
  const char *cPropertySignalName = "RSSI";
  const char *cPropertySignalDataType = "integer";
  const char *cPropertySignalFormat = "dBm";

  const char *cPropertyMac = "mac";
  const char *cPropertyMacName = "Mac Address";
  const char *cPropertyMacDataType = "sring";
  const char *cPropertyMacFormat = "";

  const char *cPropertyResetReason = "resetReason";
  const char *cPropertyResetReasonName = "Last Reset Reason";
  const char *cPropertyResetReasonDataType = "string";
  const char *cPropertyResetReasonFormat = "";

  const char *cPropertySupply = "voltage";
  const char *cPropertySupplyName = "3.3V Supply";
  const char *cPropertySupplyDataType = "float";
  const char *cPropertySupplyFormat = "V";
};
