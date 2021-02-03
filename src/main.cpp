/**
 * The Waveshare MCP23017 Module does not have a RESET LINE.
 * Thus physical power off is the only way to reset it.
 * 
 * Custom Configuration Settings
 *  - ipolA: 255 = 0xFF     Controls the input inversion for input pins
 *  - ipolB: 255 = 0xFF     zero = !inverted, one = inverted
 * 
 */

#ifdef ESP8266
extern "C"
{
#include "user_interface.h" // to set CPU Freq for Non-Huzzah's
}
#endif

#include <Homie.h>
#include "MCP23017Node.hpp"
#include "MetricsNode.hpp"

#define SKN_MOD_NAME    "HardWiredAlarms"
#define SKN_MOD_VERSION "1.3.1"
#define SKN_MOD_BRAND   "SknSensors"
#define SKN_NODE_TITLE  "Wired Sensors"
#define SKN_NODE_TYPE   "sensor"
#define SKN_NODE_ID     "wiredMonitor"

#define SKN_MNODE_TITLE "Device Info"
#define SKN_MNODE_TYPE "sensor"
#define SKN_MNODE_ID "hardware"

// Select SDA and SCL pins for I2C communication and Motion
#define PIN_SCL   D6  // 12
#define PIN_SDA   D7  // 13
#define PIN_INTA  D5  // 14
#define MCP23017_ADDR 0x27

HomieSetting<long> ipolASetting("ipolA", "Input inversion group A");
HomieSetting<long> ipolBSetting("ipolB", "Input inversion group B");

MCP23017Node mcpProvider(PIN_SDA, PIN_SCL, PIN_INTA, MCP23017_ADDR, 
                         SKN_NODE_ID, SKN_NODE_TITLE, SKN_NODE_TYPE);
MetricsNode metricsNode(SKN_MNODE_ID, SKN_MNODE_TITLE, SKN_MNODE_TYPE);

ADC_MODE(ADC_VCC); //vcc read

bool broadcastHandler(const String &level, const String &value)
{
  Homie.getLogger() << "Received broadcast level " << level << ": " << value << endl;
  return true;
}

void setup()
{

  Serial.begin(115200);
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
    .setBroadcastHandler(broadcastHandler)
    .setLedPin(LED_BUILTIN, LOW);

  yield();
  
  Homie.setup();
}

void loop()
{
  Homie.loop();
}
