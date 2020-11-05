/**
 * The Waveshare MCP23017 Module does not have a RESET LINE.
 * Thus physical power off is the only way to reset it.
 * 
 * 
 */
// #include <Arduino.h>
#include <Homie.h>
#include <brzo_i2c.h>
#include <cppQueue.h>

#define IMPLEMENTATION FIFO
#define OVERWRITE false

#define MCP23017_ADDR 0x27
#define I2C_KHZ 400
#define PIN_INT 14
#define PIN_SCL  5
#define PIN_SDA  4
#define DEPTH 22 // 22

volatile bool interruptDataLoss = false;
volatile unsigned long events = 0;
volatile bool clearInterruptCycle = false;

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

typedef struct __attribute__((packed)) _McpIState
{
  uint8_t intfA;
  uint8_t intfB;
  uint8_t intcapA;
  uint8_t intcapB;
} McpIState, *PMcpIState;

byte ICACHE_RAM_ATTR mcpClearInterrupts();
void ICACHE_RAM_ATTR interruptHandler();
void mcpSendStatus(bool statusCycle);

McpIState mcp;

Queue mcpQueue(sizeof(McpIState), 16, IMPLEMENTATION, OVERWRITE); // Instantiate queue

// id, name, type, range, lower, upper
HomieNode wiredProvider("entry", "Entry", "contact", true, 0, 15);

bool broadcastHandler(const String &level, const String &value)
{
  Homie.getLogger() << "Received broadcast level " << level << ": " << value << endl;
  return true;
}

void ICACHE_RAM_ATTR onHomieEvent(const HomieEvent &event)
{
  switch (event.type)
  {
  case HomieEventType::NORMAL_MODE:
    Serial << "Normal mode started !" << endl;
    break;
  case HomieEventType::WIFI_DISCONNECTED:
    Serial << "Wi-Fi disconnected, reason: " << (int8_t)event.wifiReason << endl;
    break;
  case HomieEventType::MQTT_READY:
    Serial << "MQTT connected !" << endl;
    break;
  case HomieEventType::MQTT_DISCONNECTED:
    Serial << "MQTT disconnected, reason: " << (int8_t)event.mqttReason << endl;
    break;
  case HomieEventType::SENDING_STATISTICS:
    Serial << "Sending statistics !" << endl;
    mcpSendStatus( true );    
    break;
  default:
    break;
  }
}

void mcpSendStatus(bool statusCycle=false) {
  HomieRange rangeA = {true, 16};
  HomieRange rangeB = {true, 16};

  Serial.printf("Event.Count[%05lu] Data.Loss[%s], Queue.Depth[%d], INTFA[" BYTE_TO_BINARY_PATTERN "] INTFB[" BYTE_TO_BINARY_PATTERN "] INTCAPA[" BYTE_TO_GPIO_PATTERN "] INTCAPB[" BYTE_TO_GPIO_PATTERN "]\n",
                events, interruptDataLoss ? "Yes" : "No", mcpQueue.getCount(),
                BYTE_TO_BINARY(mcp.intfA), BYTE_TO_BINARY(mcp.intfB),
                GPIOA_FROM_BYTE(mcp.intcapA), GPIOB_FROM_BYTE(mcp.intcapB));

  String gpioA[] = {GPIOA_FROM_BYTE(mcp.intfA)};
  String gpioB[] = {GPIOB_FROM_BYTE(mcp.intfB)};
  String intcapA[] = {BYTE_TO_STRING(mcp.intcapA)};
  String intcapB[] = {BYTE_TO_STRING(mcp.intcapB)};
  for (uint8_t idx = 0; idx < 8; idx++)
  {
    rangeA.index = (7 - idx);
    if (!gpioA[idx].equals(" ") || statusCycle)
    {
      wiredProvider.setProperty("entry").setRange(rangeA).send(intcapA[idx].equals("1") ? "true" : "false");
    }
    rangeB.index = (15 - idx);
    if (!gpioB[idx].equals(" ") || statusCycle)
    {
      wiredProvider.setProperty("entry").setRange(rangeB).send(intcapB[idx].equals("1") ? "true" : "false");
    }
  }

  clearInterruptCycle = statusCycle;
}

byte ICACHE_RAM_ATTR mcpClearInterrupts()
{
  brzo_i2c_start_transaction(MCP23017_ADDR, I2C_KHZ);
  byte regData[2] = {0x10, 0x00}; // INTCAP         -- clean any missed interrupts
  brzo_i2c_write(regData, 1, true);
  brzo_i2c_read(regData, 2, false);
  return brzo_i2c_end_transaction();
}

void  loopHandler()
{

  if (!mcpQueue.isEmpty())
    digitalWrite(LED_BUILTIN, LOW);

  while (!mcpQueue.isEmpty())
  {    
    if (mcpQueue.pop(&mcp) ) {
      mcpSendStatus( false );
      yield();
    }
  }
  interruptDataLoss = false;
  digitalWrite(LED_BUILTIN, HIGH);

  yield();

  if (clearInterruptCycle)
  {
    clearInterruptCycle = false;
    digitalWrite(LED_BUILTIN, LOW);
    mcpClearInterrupts();
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void ICACHE_RAM_ATTR interruptHandler() {
  McpIState mcpi;

  brzo_i2c_start_transaction(MCP23017_ADDR, I2C_KHZ);
  byte regData[2] = {0x0E, 0x00}; // INTF -> INTCAP
  brzo_i2c_write(regData, 1, true);
  brzo_i2c_read((uint8_t *)&(mcpi.intfA), sizeof(McpIState), false);
  if (!brzo_i2c_end_transaction())
  {
    interruptDataLoss = !mcpQueue.push(&mcpi);
  }
  events += 1;
}


byte ICACHE_RAM_ATTR mcpInit() {

  byte retCode;

  brzo_i2c_setup(PIN_SDA, PIN_SCL, 200);

  /*
   * Configure MCP23017 
  */
  byte regData[8];
  brzo_i2c_start_transaction(MCP23017_ADDR, I2C_KHZ);
  regData[0] = 0x0A; // IOCON
  regData[1] = 0x40;
  regData[2] = 0x40;
  brzo_i2c_write(regData, 3, false);
  retCode = brzo_i2c_end_transaction();
  Serial.printf("IOCON Initilization return code=%d\n", retCode);

  brzo_i2c_start_transaction(MCP23017_ADDR, I2C_KHZ);
  regData[0] = 0x02; // IPOL
  regData[1] = 0xff;
  regData[2] = 0xff;
  regData[3] = 0xff; // GPINTEN
  regData[4] = 0xff;
  brzo_i2c_write(regData, 5, false);
  retCode = brzo_i2c_end_transaction();
  Serial.printf("IPOL/GPINTEN Initilization return code=%d\n", retCode);

  brzo_i2c_start_transaction(MCP23017_ADDR, I2C_KHZ);
  regData[0] = 0x0C; // GPPU
  regData[1] = 0xff;
  regData[2] = 0xff;
  brzo_i2c_write(regData, 3, false);
  retCode = brzo_i2c_end_transaction();
  Serial.printf("GPPU Initilization return code=%d\n", retCode);

  mcpClearInterrupts();

  return retCode;
}

void ICACHE_RAM_ATTR setupHandler()
{  
  mcpInit();
  attachInterrupt(digitalPinToInterrupt(PIN_INT), interruptHandler, FALLING);
}

void setup()
{

  Serial.begin(115200);
  Serial.println("");
  Serial.printf("... Online @ %dmhz\n", ESP.getCpuFreqMHz());

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_INT, INPUT_PULLUP);
  
  Homie_setFirmware("wired-provider-16", "1.0.1");
  Homie_setBrand("SknSensors");
  Homie
      .setBroadcastHandler(broadcastHandler)
      .setLoopFunction(loopHandler)
      .setSetupFunction(setupHandler)
      .onEvent(onHomieEvent)
      .setLedPin(LED_BUILTIN, LOW);

  wiredProvider.advertise("entry")
      .setRetained(true)
      .setName("Entry[]")
      .setDatatype("boolean")
      .settable(false);

  Homie.setup();
}

void loop()
{
  Homie.loop();
}
