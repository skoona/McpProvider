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

const char bank0[][68] = {
              "IODIRA   1111 1111",
              "IODIRB   1111 1111",
              "IPOLA    0000 0000",
              "IPOLB    0000 0000",
              "GPINTENA 0000 0000",
              "GPINTENB 0000 0000",
              "DEFVALA  0000 0000",
              "DEFVALB  0000 0000",
              "INTCONA  0000 0000",
              "INTCONB  0000 0000",
              "IOCON-A  0000 0000 {BANK,MIRROR,SEQOP,DISSLW,HAEN,ODR,INTPOL,—}",
              "IOCON-B  0000 0000 {BANK,MIRROR,SEQOP,DISSLW,HAEN,ODR,INTPOL,—}",
              "GPPUA    0000 0000",
              "GPPUB    0000 0000",
              "INTFA    0000 0000",
              "INTFB    0000 0000",
              "INTCAPA  0000 0000",
              "INTCAPB  0000 0000",
              "GPIOA    0000 0000",
              "GPIOB    0000 0000",
              "OLATA    0000 0000",
              "OLATB    0000 0000"
              };

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

typedef struct __attribute__((packed)) _McpBits
{
  uint8_t p7 : 1, p6 : 1, p5 : 1, p4 : 1, p3 : 1, p2 : 1, p1 : 1, p0 : 1;
} McpBits;

typedef union __attribute__((packed)) _McpByte
{
  McpBits bit;
  uint8_t port;
} MCPByte;

typedef struct __attribute__((packed)) _McpIState
{
  MCPByte intfA;
  MCPByte intfB;
  MCPByte intcapA;
  MCPByte intcapB;
} McpIState, *PMcpIState;

byte ICACHE_RAM_ATTR mcpClearInterrupts();
void ICACHE_RAM_ATTR interruptHandler();

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
    // attachInterrupt(digitalPinToInterrupt(PIN_INT), interruptHandler, FALLING);
    // mcpClearInterrupts();
    break;
  case HomieEventType::MQTT_DISCONNECTED:
    Serial << "MQTT disconnected, reason: " << (int8_t)event.mqttReason << endl;
    // detachInterrupt(digitalPinToInterrupt(PIN_INT));
    // mcpClearInterrupts();
    break;
  case HomieEventType::SENDING_STATISTICS:
    Serial << "Sending statistics !" << endl;
        // send state of inputs
    // mcpClearInterrupts();
    break;
  default:
    break;
  }
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
  HomieRange rangeA = {true, 16};
  HomieRange rangeB = {true, 16};

  if (!mcpQueue.isEmpty())
    digitalWrite(LED_BUILTIN, LOW);

  while (!mcpQueue.isEmpty())
  {    
    mcpQueue.pop(&mcp);
    Serial.printf("Event.Count[%05lu] Data.Loss[%s], Queue.Depth[%d], INTFA[" BYTE_TO_BINARY_PATTERN "] INTFB[" BYTE_TO_BINARY_PATTERN "] INTCAPA[" BYTE_TO_GPIO_PATTERN "] INTCAPB[" BYTE_TO_GPIO_PATTERN "]\n",
                  events, interruptDataLoss ? "Yes" : "No", mcpQueue.getCount(),
                  BYTE_TO_BINARY(mcp.intfA.port), BYTE_TO_BINARY(mcp.intfB.port),
                  GPIOA_FROM_BYTE(mcp.intcapA.port), GPIOB_FROM_BYTE(mcp.intcapB.port));

    String gpioA[]   = { GPIOA_FROM_BYTE(mcp.intfA.port) };
    String gpioB[]   = { GPIOB_FROM_BYTE(mcp.intfB.port) };
    String intcapA[] = { BYTE_TO_STRING(mcp.intcapA.port) };
    String intcapB[] = { BYTE_TO_STRING(mcp.intcapB.port) };
    for (uint8_t idx = 0; idx < 8; idx++)
    {
      rangeA.index = (7 - idx);
      if ( !gpioA[idx].equals(" ") ) {
        Homie.getLogger() << "Index[" << rangeA.index << "] = " << gpioA[idx] << ", Pin Value = " << intcapA[idx] << endl;
        wiredProvider.setProperty("entry").setRange(rangeA).send( intcapA[idx].equals("1") ? "true" : "false" );
      }
      rangeB.index = (15 - idx);
      if (!gpioB[idx].equals(" "))
      {
        Homie.getLogger() << "Index[" << rangeB.index << "] = " << gpioB[idx] << ", Pin Value = " << intcapB[idx] << endl;
        wiredProvider.setProperty("entry").setRange(rangeB).send(intcapB[idx].equals("1") ? "true" : "false" );
      }
    }
  }
  interruptDataLoss = false;
  digitalWrite(LED_BUILTIN, HIGH);

  // if ((millis() % 60000UL) == 0)
  // {
  //   digitalWrite(LED_BUILTIN, LOW);
  //   mcpClearInterrupts();
  //   digitalWrite(LED_BUILTIN, HIGH);
  //   Serial.print(".");
  // }
  yield();
}

void ICACHE_RAM_ATTR interruptHandler() {
  McpIState mcpi;

  brzo_i2c_start_transaction(MCP23017_ADDR, I2C_KHZ);
  byte regData[2] = {0x0E, 0x00}; // INTF -> INTCAP
  brzo_i2c_write(regData, 1, true);
  brzo_i2c_read((uint8_t *)&(mcpi.intfA.port), sizeof(McpIState), false);
  if (!brzo_i2c_end_transaction())
  {
    interruptDataLoss = !mcpQueue.push(&mcpi);
  }
  events += 1;
}


byte ICACHE_RAM_ATTR mcpInit() {
  uint8_t before[DEPTH];
  uint8_t after[DEPTH];

  byte retCode;

  brzo_i2c_setup(PIN_SDA, PIN_SCL, 200);

  /*
   * Document PowerOn State
  */
  brzo_i2c_start_transaction(MCP23017_ADDR, I2C_KHZ);
  before[0] = 0x00;
  brzo_i2c_write(before, 1, true);     // BANK.0.IODIRA
  brzo_i2c_read(before, DEPTH, false); // BANK.0.ALL
  retCode = brzo_i2c_end_transaction();
  Serial.printf("Before return code=%d\n", retCode);
  for (int idx = 0; idx < DEPTH; idx++)
  {
    Serial.printf("reg[0x%02x] %s value[" BYTE_TO_BINARY_PATTERN "]\n", idx, &bank0[idx][0], BYTE_TO_BINARY(before[idx]));
  }

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

  /*
   * Confirm Configuration
  */
  brzo_i2c_start_transaction(MCP23017_ADDR, I2C_KHZ);
  after[0] = 0x00;
  brzo_i2c_write(after, 1, true);
  brzo_i2c_read(after, DEPTH, false);
  retCode = brzo_i2c_end_transaction();
  Serial.printf("After return code=%d\n", retCode);
  for (int idx = 0; idx < DEPTH; idx++)
  {
    Serial.printf("reg[0x%02x] %s value[" BYTE_TO_BINARY_PATTERN "]\n", idx, &bank0[idx][0], BYTE_TO_BINARY(after[idx]));
  }

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
  
  Homie_setFirmware("wired-provider-16", "1.0.0");
  Homie_setBrand("SknSensors");
  Homie
      .setBroadcastHandler(broadcastHandler)
      .setLoopFunction(loopHandler)
      .setSetupFunction(setupHandler)
      .onEvent(onHomieEvent)
      .setLedPin(LED_BUILTIN, LOW);

  wiredProvider.advertise("entry")
          .setName("Entry[]")
          .setDatatype("boolean")
          .settable(false);

  Homie.setup();
}

void loop()
{
  Homie.loop();
}
