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
      (byte & 0x80 ? "1" : "0"),     \
      (byte & 0x40 ? "1" : "0"), \
      (byte & 0x20 ? "1" : "0"), \
      (byte & 0x10 ? "1" : "0"), \
      (byte & 0x08 ? "1" : "0"), \
      (byte & 0x04 ? "1" : "0"), \
      (byte & 0x02 ? "1" : "0"), \
      (byte & 0x01 ? "1" : "0")

#define BYTE_TO_BINARY(byte)     \
      (byte & 0x80 ? '1' : '0'),     \
      (byte & 0x40 ? '1' : '0'), \
      (byte & 0x20 ? '1' : '0'), \
      (byte & 0x10 ? '1' : '0'), \
      (byte & 0x08 ? '1' : '0'), \
      (byte & 0x04 ? '1' : '0'), \
      (byte & 0x02 ? '1' : '0'), \
      (byte & 0x01 ? '1' : '0')

#define GPIOA_FROM_BYTE(byte)      \
      (byte & 0x80 ? "7" : " "), \
      (byte & 0x40 ? "6" : " "), \
      (byte & 0x20 ? "5" : " "), \
      (byte & 0x10 ? "4" : " "), \
      (byte & 0x08 ? "3" : " "), \
      (byte & 0x04 ? "2" : " "), \
      (byte & 0x02 ? "1" : " "), \
      (byte & 0x01 ? "0" : " ")

#define GPIOB_FROM_BYTE(byte)     \
      (byte & 0x80 ? "15" : " "),     \
      (byte & 0x40 ? "14" : " "), \
      (byte & 0x20 ? "13" : " "), \
      (byte & 0x10 ? "12" : " "), \
      (byte & 0x08 ? "11" : " "), \
      (byte & 0x04 ? "10" : " "), \
      (byte & 0x02 ? "9" : " "), \
      (byte & 0x01 ? "8" : " ")

MCP23017Node::MCP23017Node(const uint8_t sdaPin, const uint8_t sclPin, const uint8_t isrPin, const uint8_t devAddr, const char *id, const char *name, const char *nType, const int measurementInterval)    : HomieNode(id, name, nType, false, 0U, 0U), 
    _sdaPin(sdaPin), 
    _sclPin(sclPin),
    _isrPin(isrPin),
    _devAddr(devAddr)
{

  _measurementInterval = (measurementInterval > MIN_INTERVAL) ? measurementInterval : MIN_INTERVAL;
  _lastMeasurement = 0;

  // Start up
  mcpQueue = new cppQueue(sizeof(McpIState), 16, IMPLEMENTATION, OVERWRITE); // Instantiate queue

  pinMode(_isrPin, INPUT_PULLUP);
}

void MCP23017Node::handleCurrentState(McpIState *mcp, bool statusOverride)
{
  int labelA = 0, labelB = 0;
  if (statusOverride) {
    readState();
  }

  Serial.printf("Source:[%s] Event.Count[%05lu] Data.Loss[%s], Queue.Depth[%d], INTFA[" BYTE_TO_BINARY_PATTERN "] INTFB[" BYTE_TO_BINARY_PATTERN "] INTCAPA[" BYTE_TO_GPIO_PATTERN "] INTCAPB[" BYTE_TO_GPIO_PATTERN "]\n",
                statusOverride ? "Status" : "ISR   ",
                events, interruptDataLoss ? "Yes" : "No", mcpQueue->getCount(),
                BYTE_TO_BINARY(mcp->intfA), BYTE_TO_BINARY(mcp->intfB),
                GPIOA_FROM_BYTE(mcp->intcapA), GPIOB_FROM_BYTE(mcp->intcapB));

  String gpioA[] = {GPIOA_FROM_BYTE(mcp->intfA)};
  String gpioB[] = {GPIOB_FROM_BYTE(mcp->intfB)};
  String intcapA[] = {BYTE_TO_STRING(mcp->intcapA)};
  String intcapB[] = {BYTE_TO_STRING(mcp->intcapB)};

  for (uint8_t idx = 0; idx < 8; idx++)
  {
    labelA = (7 - idx);
    labelB = (15 - idx);

    if (!gpioA[idx].equals(" ") || statusOverride)
    {
      setProperty(cProperty[labelA])
          .setRetained(true)
          .send(intcapA[idx].equals("1") ? "CLOSED" : "OPEN");
    }
    if (!gpioB[idx].equals(" ") || statusOverride)
    {
      setProperty(cProperty[labelB])
          .setRetained(true)
          .send(intcapB[idx].equals("1") ? "CLOSED" : "OPEN");
    }
  }
}

void ICACHE_RAM_ATTR MCP23017Node::interruptHandler() {
  McpIState mcpi;

  brzo_i2c_start_transaction(_devAddr, I2C_KHZ);
  byte regData[2] = {MCP23017_INTFA, 0x00}; // INTF -> INTCAP
  brzo_i2c_write(regData, 1, true);
  brzo_i2c_read((uint8_t *)&(mcpi.intfA), sizeof(McpIState), false);
  if (!brzo_i2c_end_transaction())
  {
    interruptDataLoss = !mcpQueue->push(&mcpi);
  }
  _isrLastTriggeredAt = millis();
  events += 1;
}
byte ICACHE_RAM_ATTR MCP23017Node::readState()
{
  brzo_i2c_start_transaction(_devAddr, I2C_KHZ);
  byte regData[2] = {MCP23017_GPIOA, 0x00}; // GPIOA/B
  brzo_i2c_write(regData, 1, true);
  brzo_i2c_read((uint8_t *)&(mcp.intcapA), sizeof(2), false);
  return brzo_i2c_end_transaction();
}

byte ICACHE_RAM_ATTR MCP23017Node::mcpClearInterrupts() {
  uint8_t res;
  brzo_i2c_start_transaction(_devAddr, I2C_KHZ);
  byte regData[2] = {MCP23017_INTCAPA, 0x00}; // INTCAP         -- clean any missed interrupts
  brzo_i2c_write(regData, 1, true);
  brzo_i2c_read(regData, 2, false);
  res = brzo_i2c_end_transaction();
  _isrLastTriggeredAt = millis();
  return res;
}

byte ICACHE_RAM_ATTR MCP23017Node::mcpInit() {
  byte retCode;

  brzo_i2c_setup(_sdaPin, _sclPin, 200); // clock_stretch_time_out_usec
  
  /*
   * Configure MCP23017 
  */
  byte regData[8];
  brzo_i2c_start_transaction(_devAddr, I2C_KHZ);
  regData[0] = MCP23017_IOCONA; // IOCON - Turn on mirroring for ISRs
  regData[1] = 0x40;
  regData[2] = 0x40;
  brzo_i2c_write(regData, 3, false);
  retCode = brzo_i2c_end_transaction();
  Serial.printf("IOCON Initilization return code=%d\n", retCode);

  Serial.print("ipolA: 0x"); Serial.println(_ipolASetting, HEX);
  Serial.print("ipolB: 0x"); Serial.println(_ipolBSetting, HEX);

  brzo_i2c_start_transaction(_devAddr, I2C_KHZ);
  regData[0] = MCP23017_IPOLA; // IPOL - custom bit flips
  regData[1] = _ipolASetting;
  regData[2] = _ipolBSetting;
  regData[3] = 0xff; // GPINTEN - enable interrupts on change for all
  regData[4] = 0xff;
  brzo_i2c_write(regData, 5, false);
  retCode = brzo_i2c_end_transaction();
  Serial.printf("IPOL/GPINTEN Initilization return code=%d\n", retCode);

  brzo_i2c_start_transaction(_devAddr, I2C_KHZ);
  regData[0] = MCP23017_GPPUA; // GPPU Turn Pullups ON for all Pins
  regData[1] = 0xff;
  regData[2] = 0xff;
  brzo_i2c_write(regData, 3, false);
  retCode = brzo_i2c_end_transaction();
  Serial.printf("GPPU Initilization return code=%d\n", retCode);

  return retCode;
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
        .setFormat(cPropertyBaseFormat);
  }

  mcpInit();
  mcpClearInterrupts();
}

/**
 * Called by Homie when homie is connected and in run mode
*/
void MCP23017Node::loop() {
  _isrTrigger = digitalRead(_isrPin);
  if (!_isrTrigger) {                // active low
    _isrTriggeredAt = millis();      // note when it happened

    // limit repeats to greater than 80 ms
    if ((_isrTriggeredAt - _isrLastTriggeredAt) >= _InterruptBounce)
    {
      interruptHandler();              // read device data
    } else {
      mcpClearInterrupts();            // clears _isrTriggeredAt
    }
    
    _isrTrigger = HIGH;
    return;
  }

  if ( (millis() - _lastMeasurement >= (_measurementInterval * 1000UL) ) 
      || (_lastMeasurement == 0))
  {
    _lastMeasurement = millis();

    handleCurrentState(&mcp, true);

    yield();
  }

  if ( !mcpQueue->isEmpty() )
  {
    digitalWrite(LED_BUILTIN, LOW);

    while (!mcpQueue->isEmpty())
    {
      if (mcpQueue->pop(&mcp) ) {
        handleCurrentState( &mcp, false);
      }
      yield();
    }
    interruptDataLoss = false;
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

