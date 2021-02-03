/**
 * Homie Node to overcome OpenHAB's lack of support for device metrics
 * 
 */
#include "MetricsNode.hpp"

MetricsNode::MetricsNode(const char *id, const char *name, const char *nType, const int measurementInterval)
    : HomieNode(id, name, nType, false, 0U, 0U)
{
  _measurementInterval = (measurementInterval > MIN_INTERVAL) ? measurementInterval : MIN_INTERVAL;
  _lastMeasurement     = 0;

}


/**
    * Called by Homie when Homie.setup() is called; Once!
  */
void MetricsNode::setup()
{
  Homie.getLogger() << cCaption
                    << endl;
  Homie.getLogger() << cIndent
                    << F("RSSI: ")
                    << WiFi.RSSI()
                    << F(" MAC: ")
                    << WiFi.macAddress().c_str()
                    << F(" Reset Reason: ")
                    << ESP.getResetReason().c_str()
                    << endl;

  advertise(cPropertySignal)
      .setName(cPropertySignalName)
      .setDatatype(cPropertySignalDataType)
      .setUnit(cPropertySignalFormat);

  advertise(cPropertyMac)
      .setName(cPropertyMac)
      .setDatatype(cPropertyMacDataType)
      .setUnit(cPropertyMacFormat);

  advertise(cPropertyResetReason)
      .setName(cPropertyResetReasonName)
      .setDatatype(cPropertyResetReasonDataType)
      .setUnit(cPropertyResetReasonFormat);

  advertise(cPropertySupply)
      .setName(cPropertySupplyName)
      .setDatatype(cPropertySupplyDataType)
      .setUnit(cPropertySupplyFormat);
}

  /**
   * Called by Homie when homie is connected and in run mode
  */
  void MetricsNode::loop() {
    if (millis() - _lastMeasurement >= (_measurementInterval * 1000UL) || _lastMeasurement == 0) {
      _lastMeasurement = millis();

      Homie.getLogger() << cIndent 
                        << F("〽 Sending Device Metrics: ") 
                        << getId() 
                        << endl;
      Homie.getLogger() << cIndent
                        << F("RSSI: ")
                        << WiFi.RSSI()
                        << endl;
      Homie.getLogger() << cIndent
                        << F(" MAC: ")
                        << WiFi.macAddress().c_str()
                        << endl;
      Homie.getLogger() << cIndent
                        << F(" Reset Reason: ")
                        << ESP.getResetReason().c_str()
                        << endl;
      Homie.getLogger() << cIndent
                        << F(" 3.3V Supply: ")
                        << (ESP.getVcc() / 1000.0)
                        << endl;

      setProperty(cPropertySignal)
          .setRetained(true)
          .send(String(WiFi.RSSI()));

      setProperty(cPropertyMac)
          .setRetained(true)
          .send(WiFi.macAddress());

      setProperty(cPropertyResetReason)
          .setRetained(true)
          .send(ESP.getResetReason());

      setProperty(cPropertySupply)
          .setRetained(true)
          .send(String((ESP.getVcc() / 1000.0)));
    }
  }
