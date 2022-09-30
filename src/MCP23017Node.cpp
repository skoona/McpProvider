/**
 * Homie Node for MCP23017 I2c Port Expander
 * 
 */

#include "MCP23017Node.hpp"

// Queue values
#define IMPLEMENTATION FIFO
#define OVERWRITE false

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_GPIO_PATTERN "%s,%s,%s,%s,%s,%s,%s,%s"

#define BYTE_TO_STRING(byte)     \
      (byte & 0x80 ? "1" : "0"), \
      (byte & 0x40 ? "1" : "0"), \
      (byte & 0x20 ? "1" : "0"), \
      (byte & 0x10 ? "1" : "0"), \
      (byte & 0x08 ? "1" : "0"), \
      (byte & 0x04 ? "1" : "0"), \
      (byte & 0x02 ? "1" : "0"), \
      (byte & 0x01 ? "1" : "0")

#define BYTE_TO_BINARY(byte)     \
      (byte & 0x80 ? '1' : '0'), \
      (byte & 0x40 ? '1' : '0'), \
      (byte & 0x20 ? '1' : '0'), \
      (byte & 0x10 ? '1' : '0'), \
      (byte & 0x08 ? '1' : '0'), \
      (byte & 0x04 ? '1' : '0'), \
      (byte & 0x02 ? '1' : '0'), \
      (byte & 0x01 ? '1' : '0')

#define GPIOA_FROM_BYTE(byte)    \
      (byte & 0x80 ? "7" : " "), \
      (byte & 0x40 ? "6" : " "), \
      (byte & 0x20 ? "5" : " "), \
      (byte & 0x10 ? "4" : " "), \
      (byte & 0x08 ? "3" : " "), \
      (byte & 0x04 ? "2" : " "), \
      (byte & 0x02 ? "1" : " "), \
      (byte & 0x01 ? "0" : " ")

#define GPIOB_FROM_BYTE(byte)     \
      (byte & 0x80 ? "15" : " "), \
      (byte & 0x40 ? "14" : " "), \
      (byte & 0x20 ? "13" : " "), \
      (byte & 0x10 ? "12" : " "), \
      (byte & 0x08 ? "11" : " "), \
      (byte & 0x04 ? "10" : " "), \
      (byte & 0x02 ? "9" : " "), \
      (byte & 0x01 ? "8" : " ")

MCP23017Node::MCP23017Node(const uint8_t sdaPin, const uint8_t sclPin, const uint8_t isrPin, const uint8_t devAddr, const char *id, const char *name, const char *nType, const int measurementInterval): 
    HomieNode(id, name, nType, false, 0U, 0U), 
    _sdaPin(sdaPin), 
    _sclPin(sclPin),
    _isrPin(isrPin),
    _devAddr(devAddr),
    mcp(devAddr, Wire) {
  
  _measurementInterval = (measurementInterval > MIN_INTERVAL) ? measurementInterval : MIN_INTERVAL;
  _lastMeasurement = 0;
  
  pinMode(_isrPin, INPUT_PULLUP);
}

void MCP23017Node::publishCurrentStatus(bool readStatus)
{
  int labelA = 0, labelB = 0;
  if (readStatus) {
    mcp.readRegister(MCP23017Register::GPIO_A, mcpState.intcapA, mcpState.intcapB);
  }

  Serial.printf(" ✖ Source:[%s] Event.Count[%05lu] Data.Loss[%s], INTFA[" BYTE_TO_BINARY_PATTERN "] INTFB[" BYTE_TO_BINARY_PATTERN "] INTCAPA[" BYTE_TO_GPIO_PATTERN "] INTCAPB[" BYTE_TO_GPIO_PATTERN "]\n",
                readStatus ? "Status" : "ISR   ",
                events, interruptDataLoss ? "Yes" : "No", 
                BYTE_TO_BINARY(mcpState.intfA), BYTE_TO_BINARY(mcpState.intfB),
                GPIOA_FROM_BYTE(mcpState.intcapA), GPIOB_FROM_BYTE(mcpState.intcapB));

  String gpioA[] = {GPIOA_FROM_BYTE(mcpState.intfA)};
  String gpioB[] = {GPIOB_FROM_BYTE(mcpState.intfB)};
  String intcapA[] = {BYTE_TO_STRING(mcpState.intcapA)};
  String intcapB[] = {BYTE_TO_STRING(mcpState.intcapB)};

  if(!gRun) return;  // if network is offline, don't try to send updates
  for (uint8_t idx = 0; idx < 8; idx++)
  {
    labelA = (7 - idx);
    labelB = (15 - idx);

    if (!gpioA[idx].equals(" ") || readStatus)
    {
      setProperty(cProperty[labelA])
          .setRetained(true)
          .send(intcapA[idx].equals("1") ? "CLOSED" : "OPEN");
    }
    if (!gpioB[idx].equals(" ") || readStatus)
    {
      setProperty(cProperty[labelB])
          .setRetained(true)
          .send(intcapB[idx].equals("1") ? "CLOSED" : "OPEN");
    }
  }
}

void IRAM_ATTR MCP23017Node::handleChangeInterrupts()
{
  if(!mcp.interruptedBy(mcpState.intfA, mcpState.intfB)) {
    mcp.readRegister(MCP23017Register::GPIO_A, mcpState.intcapA, mcpState.intcapB);
    interruptDataLoss = false;
  } else {
    interruptDataLoss = true;
  }
  _isrLastTriggeredAt = millis();
  events += 1;
  mcp.clearInterrupts();
}

byte IRAM_ATTR MCP23017Node::begin()
{
  // Start up
  byte retCode;

  Wire.begin(_sdaPin, _sclPin, _devAddr);

  /*
   * Configure MCP23017 
   * BANK   = 	0 : sequential register addresses
	 * MIRROR = 	1 : use configureInterrupt 
	 * SEQOP  = 	0 : sequential operation disabled, address pointer does not increment
	 * DISSLW = 	0 : slew rate enabled
	 * HAEN   = 	0 : hardware address pin is always enabled on 23017
	 * ODR    = 	0 : open drain output
	 * INTPOL = 	0 : interrupt active low, high
   * N/A    = 	0 : Not Used
   * 
  */
  
  retCode = mcp.writeRegister(MCP23017Register::IOCON, 0x42, 0x42);           // Enable MIRROR interrupt mode
  Serial.printf(" ✖ IOCON Initilization return code=%d\n", retCode);

  retCode |= mcp.writeRegister(MCP23017Register::IODIR_A, 0xff, 0xff);        // IODIR set for Input
  Serial.printf(" ✖ IODIR Initilization return code=%d\n", retCode);

  retCode |= mcp.writeRegister(MCP23017Register::GPPU_A, 0xff, 0xff);         // GPPU set for Pullup
  Serial.printf(" ✖ GPPU Initilization return code=%d\n", retCode);

  retCode |= mcp.writeRegister(MCP23017Register::IPOL_A, _ipolASetting, _ipolBSetting);  // IPOL set for Inverted Polarity
  Serial.printf(" ✖ IPOL Initilization return code=%d\n", retCode);

  retCode |= mcp.writeRegister(MCP23017Register::GPINTEN_A, 0xff, 0xff);       //  GPINTEN set for Interrupt on change
  Serial.printf(" ✖ GPINTEN Initilization return code=%d\n", retCode);

  retCode |= mcp.writeRegister(MCP23017Register::DEFVAL_A, 0xff, 0xff);        // DEFVAL set to Interrupt if pin different from buffer
  Serial.printf(" ✖ DEFVAL Initilization return code=%d\n", retCode);

  retCode |= mcp.clearInterrupts();
  Serial.printf(" ✖ begin() clear Interrupts return code=%d\n", retCode);

  return retCode;
}

/**
 *
 */
void MCP23017Node::onReadyToOperate() {

  if(_Once) { return; }  // guard against being called twice

  Homie.getLogger()
      << "✖  "
      << "Node: " << getName()
      << " Ready to operate " 
      << endl;

  begin();

  for(int idx = 0; idx < _numPins; idx++) {
    snprintf(cProperty[idx], sizeof(cProperty[idx]), "%s%d", cPropertyBase, idx);
    snprintf(cPropertyName[idx], sizeof(cPropertyName[idx]), "%s%d", cPropertyBaseName, idx);
    advertise(cProperty[idx])
        .setName(cPropertyName[idx])
        .setDatatype(cPropertyBaseDataType)
        .setFormat(cPropertyBaseFormat)
        .setRetained(true);
  }

  _Once = true;
}

/**
 * Called by Homie when Homie.setup() is called; Once!
*/
void MCP23017Node::setup() {
  Homie.getLogger() << cCaption << endl;
  Homie.getLogger() << cIndent << cPropertyBaseName << endl;

  for(int idx = 0; idx < _numPins; idx++) {
    snprintf(cProperty[idx], sizeof(cProperty[idx]), "%s%d", cPropertyBase, idx);
    snprintf(cPropertyName[idx], sizeof(cPropertyName[idx]), "%s%d", cPropertyBaseName, idx);
    advertise(cProperty[idx])
        .setName(cPropertyName[idx])
        .setDatatype(cPropertyBaseDataType)
        .setFormat(cPropertyBaseFormat)
        .setRetained(true);
  }
}

/**
 * Called by Homie when homie is connected and in run mode
*/
void MCP23017Node::loop() {
  if (digitalRead(_isrPin)) {                // active low
    _isrTriggeredAt = millis();      // note when it happened

    // limit repeats to greater than 80 ms
    if ((_isrTriggeredAt - _isrLastTriggeredAt) >= _InterruptBounce)
    {
      handleChangeInterrupts();              // read device data
      publishCurrentStatus(false);
    } else {
      mcp.clearInterrupts();            // clears _isrTriggeredAt
      _isrLastTriggeredAt = millis();
    }
    
    return;
  }

  /*
   * send notifications every x ms */
  if ( (millis() - _lastMeasurement >= (_measurementInterval * 1000UL) ) || (_lastMeasurement == 0))
  {
    _lastMeasurement = millis();

    publishCurrentStatus(true);

    yield();
  }

  if(interruptDataLoss) {
    digitalWrite(LED_BUILTIN, !interruptDataLoss);
  } else {
    digitalWrite(LED_BUILTIN, !interruptDataLoss);
  }
}

