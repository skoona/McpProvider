/**
 * The Waveshare MCP23017 Module does not have a RESET LINE.
 * Thus physical power off is the only way to reset it.
 * 
 * Custom Configuration Settings
 *  - ipolA: 255 = 0xFF     Controls the input inversion for input pins
 *  - ipolB: 255 = 0xFF     zero = !inverted, one = inverted
 * 
 */
#include <Homie.h>
#include "MCP23017Node.hpp"
#include "MetricsNode.hpp"

#ifdef ESP8266
extern "C"
{
#include "user_interface.h" // to set CPU Freq for Non-Huzzah's
}
#endif

#define SKN_MOD_NAME    "WiredProvider"
#define SKN_MOD_VERSION "3.0.0"
#define SKN_MOD_BRAND   "SknSensors"

#define SKN_NODE_TITLE  "Wired Sensors"
#define SKN_NODE_TYPE   "contact"
#define SKN_NODE_ID     "wiredMonitor"

#define SKN_MNODE_TITLE "Device Info"
#define SKN_MNODE_TYPE "sensor"
#define SKN_MNODE_ID "hardware"

// Select SDA and SCL pins for I2C communication and Motion
#define PIN_SCL   D6  // 12
#define PIN_SDA   D7  // 13
#define PIN_INTA  D5  // 14
#define MCP23017_ADDR 0x27

ADC_MODE(ADC_VCC); //vcc read

/* 
 * Homie Settings */
HomieSetting<long> ipolASetting("ipolA", "Input inversion group A");
HomieSetting<long> ipolBSetting("ipolB", "Input inversion group B");

/* 
 * Homie Nodes */
MCP23017Node mcpProvider(PIN_SDA, PIN_SCL, PIN_INTA, MCP23017_ADDR, 
                         SKN_NODE_ID, SKN_NODE_TITLE, SKN_NODE_TYPE);

MetricsNode metricsNode(SKN_MNODE_ID, SKN_MNODE_TITLE, SKN_MNODE_TYPE);


/* *
  *
  * guard-flag to prevent sending properties when mqtt is offline
  * 
*/
volatile bool gRun=false;

/**
 * look for events that block sending property info */
void onHomieEvent(const HomieEvent& event) {
  switch (event.type) {
    case HomieEventType::MQTT_READY:
      Serial << "MQTT connected" << endl;
      gRun=true;
      break;
    case HomieEventType::MQTT_DISCONNECTED:
      Serial << "MQTT disconnected, reason: " << (int8_t)event.mqttReason << endl;
      gRun=false;
      break;
    case HomieEventType::SENDING_STATISTICS:
      Serial << "Sending statistics" << endl;
      mcpProvider.sendCurrentStatus();
      break;
    case HomieEventType::OTA_STARTED:
      gRun=false;
      break;
    case HomieEventType::OTA_SUCCESSFUL:
    case HomieEventType::OTA_FAILED:
      gRun=true;
      break;
  }
}

/*
 * Arduino Setup: Initialze Homie */
void setup()
{

  delay(100);

  // REG_SET_BIT(0x3ff00014, BIT(0));
  // REG_CLR_BIT(0x3ff00014, BIT(0));
  // system_update_cpu_freq(160);

  delay(1000);
  Serial.begin(115200);
  delay(100);
  if (!Serial)  {
    Homie.disableLogging();
  }

  Serial.println("");
  Serial.printf("... Online @ %dmhz\n", ESP.getCpuFreqMHz());

  ipolASetting.setDefaultValue(255).setValidator([](long value) {
    return value < 256;
  }); 
  ipolBSetting.setDefaultValue(255).setValidator([](long value) {
    return value < 256;
  });
  mcpProvider.setInversionPolarityAB((uint8_t)ipolASetting.get(), (uint8_t)ipolBSetting.get());
  mcpProvider.setMeasurementInterval( 60 );

  Homie_setFirmware(SKN_MOD_NAME, SKN_MOD_VERSION);
  Homie_setBrand(SKN_MOD_BRAND);

  Homie
    .setLedPin(LED_BUILTIN, LOW)
    .disableResetTrigger()
    .onEvent(onHomieEvent);

  Homie.setup();
}

/*
 * Arduino Loop: Cycles Homie Nodes */
void loop() {
  Homie.loop();
}
