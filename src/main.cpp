/**
 * The Waveshare MCP23017 Module does not have a RESET LINE.
 * Thus physical power off is the only way to reset it.
 * 
 * 
 */
#include <Arduino.h>
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

uint8_t before[DEPTH];
uint8_t after[DEPTH];


byte    retCode;

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
      (byte & 0x80 ? "PA7" : ""), \
      (byte & 0x40 ? "PA6" : ""), \
      (byte & 0x20 ? "PA5" : ""), \
      (byte & 0x10 ? "PA4" : ""), \
      (byte & 0x08 ? "PA3" : ""), \
      (byte & 0x04 ? "PA2" : ""), \
      (byte & 0x02 ? "PA1" : ""), \
      (byte & 0x01 ? "PA0" : "")
#define GPIOB_FROM_BYTE(byte)      \
      (byte & 0x80 ? "PB7" : ""), \
      (byte & 0x40 ? "PB6" : ""), \
      (byte & 0x20 ? "PB5" : ""), \
      (byte & 0x10 ? "PB4" : ""), \
      (byte & 0x08 ? "PB3" : ""), \
      (byte & 0x04 ? "PB2" : ""), \
      (byte & 0x02 ? "PB1" : ""), \
      (byte & 0x01 ? "PB0" : "")



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

Queue mcpQueue(sizeof(McpIState), 16, IMPLEMENTATION, OVERWRITE); // Instantiate queue

byte ICACHE_RAM_ATTR mcpClearInterrupts()
{
  brzo_i2c_start_transaction(MCP23017_ADDR, I2C_KHZ);
  byte regData[2] = {0x10, 0x00}; // INTCAP         -- clean any missed interrupts
  brzo_i2c_write(regData, 1, true);
  brzo_i2c_read(regData, 2, false);
  return brzo_i2c_end_transaction();
}

void ICACHE_RAM_ATTR triggerHandler() {
  McpIState mcpi;

  brzo_i2c_start_transaction(MCP23017_ADDR, I2C_KHZ);
  byte regData[2] = {0x0E, 0x00}; // INTF -> INTCAP
  brzo_i2c_write(regData, 1, true);
  brzo_i2c_read((uint8_t *)&(mcpi.intfA.port), sizeof(McpIState), false);
  if (!brzo_i2c_end_transaction())
  {
    interruptDataLoss = !mcpQueue.push(&mcpi);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.printf("... Online @ %dmhz\n", ESP.getCpuFreqMHz());

  brzo_i2c_setup(PIN_SDA, PIN_SCL, 200);
  
    /*
     * Document PowerOn State
    */
    brzo_i2c_start_transaction(MCP23017_ADDR, I2C_KHZ);
    before[0] = 0x00;
    brzo_i2c_write(before, 1, true);    // BANK.0.IODIRA
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
    brzo_i2c_write(after, 1, false);
    brzo_i2c_read(after, DEPTH, false);
    retCode = brzo_i2c_end_transaction();
    Serial.printf("After return code=%d\n", retCode);
    for (int idx = 0; idx < DEPTH; idx++)
    {
      Serial.printf("reg[0x%02x] %s value[" BYTE_TO_BINARY_PATTERN "]\n", idx, &bank0[idx][0], BYTE_TO_BINARY(after[idx]));
    }

    pinMode(PIN_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_INT), triggerHandler, FALLING);

    mcpClearInterrupts();
  }

void ICACHE_RAM_ATTR loop()
{
  McpIState mcp;

  while ( !mcpQueue.isEmpty() )
  {
    mcpQueue.pop(&mcp);
    Serial.printf("Potential Data Loss:%s, Queue.Depth=%d, INTFA[" BYTE_TO_BINARY_PATTERN "] INTFB[" BYTE_TO_BINARY_PATTERN "] INTCAPA[" BYTE_TO_GPIO_PATTERN "] INTCAPB[" BYTE_TO_GPIO_PATTERN "]\n",
                  interruptDataLoss ? "Yes" : "No", mcpQueue.getCount(),
                  BYTE_TO_BINARY(mcp.intfA.port), BYTE_TO_BINARY(mcp.intfB.port),
                  GPIOA_FROM_BYTE(mcp.intcapA.port), GPIOA_FROM_BYTE(mcp.intcapB.port));
  }
  interruptDataLoss = false;

  if ((millis() % 60000UL) == 0)
  {
    mcpClearInterrupts();
    Serial.print(".");
  }
  yield();
}

/* OUPUT

Before return code=0
reg[0x00] IODIRA   1111 1111 value[11111111]
reg[0x01] IODIRB   1111 1111 value[11111111]
reg[0x02] IPOLA    0000 0000 value[00000000]
reg[0x03] IPOLB    0000 0000 value[00000000]
reg[0x04] GPINTENA 0000 0000 value[00000000]
reg[0x05] GPINTENB 0000 0000 value[00000000]
reg[0x06] DEFVALA  0000 0000 value[00000000]
reg[0x07] DEFVALB  0000 0000 value[00000000]
reg[0x08] INTCONA  0000 0000 value[00000000]
reg[0x09] INTCONB  0000 0000 value[00000000]
reg[0x0a] IOCON-A  0000 0000 {BANK,MIRROR,SEQOP,DISSLW,HAEN,ODR,INTPOL,—} value[00000000]
reg[0x0b] IOCON-B  0000 0000 {BANK,MIRROR,SEQOP,DISSLW,HAEN,ODR,INTPOL,—} value[00000000]
reg[0x0c] GPPUA    0000 0000 value[00000000]
reg[0x0d] GPPUB    0000 0000 value[00000000]
reg[0x0e] INTFA    0000 0000 value[00000000]
reg[0x0f] INTFB    0000 0000 value[00000000]
reg[0x10] INTCAPA  0000 0000 value[00000000]
reg[0x11] INTCAPB  0000 0000 value[00000000]
reg[0x12] GPIOA    0000 0000 value[01000000]
reg[0x13] GPIOB    0000 0000 value[00000000]
reg[0x14] OLATA    0000 0000 value[00000000]
reg[0x15] OLATB    0000 0000 value[00000000]
IOCON Initilization return code=0
IPOL/GPINTEN Initilization return code=0
GPPU Initilization return code=0
After return code=0
reg[0x00] IODIRA   1111 1111 value[11111111]
reg[0x01] IODIRB   1111 1111 value[11111111]
reg[0x02] IPOLA    0000 0000 value[11111111]
reg[0x03] IPOLB    0000 0000 value[11111111]
reg[0x04] GPINTENA 0000 0000 value[11111111]
reg[0x05] GPINTENB 0000 0000 value[11111111]
reg[0x06] DEFVALA  0000 0000 value[00000000]
reg[0x07] DEFVALB  0000 0000 value[00000000]
reg[0x08] INTCONA  0000 0000 value[00000000]
reg[0x09] INTCONB  0000 0000 value[00000000]
reg[0x0a] IOCON-A  0000 0000 {BANK,MIRROR,SEQOP,DISSLW,HAEN,ODR,INTPOL,—} value[01000000]
reg[0x0b] IOCON-B  0000 0000 {BANK,MIRROR,SEQOP,DISSLW,HAEN,ODR,INTPOL,—} value[01000000]
reg[0x0c] GPPUA    0000 0000 value[11111111]
reg[0x0d] GPPUB    0000 0000 value[11111111]
reg[0x0e] INTFA    0000 0000 value[11111111]
reg[0x0f] INTFB    0000 0000 value[11111111]
reg[0x10] INTCAPA  0000 0000 value[10111111]
reg[0x11] INTCAPB  0000 0000 value[11111111]
reg[0x12] GPIOA    0000 0000 value[00111000]
reg[0x13] GPIOB    0000 0000 value[00000000]
reg[0x14] OLATA    0000 0000 value[00000000]
reg[0x15] OLATB    0000 0000 value[00000000]
GPIO return code=0, INTFA[00000000] INTFB[00000000] INTCAPA[00111000] INTCAPB[11000000] GPIOA[00111000] GPIOB[00000000] OLATA[00000000] OLATB[00000000]
GPIO return code=0, INTFA[00000000] INTFB[00000000] INTCAPA[00111000] INTCAPB[11000000] GPIOA[00111000] GPIOB[00000000] OLATA[00000000] OLATB[00000000]
GPIO return code=0, INTFA[00000000] INTFB[00000000] INTCAPA[00111000] INTCAPB[11000000] GPIOA[00111000] GPIOB[00000000] OLATA[00000000] OLATB[00000000]
GPIO return code=0, INTFA[00000000] INTFB[00000000] INTCAPA[00111000] INTCAPB[11000000] GPIOA[00111000] GPIOB[00000000] OLATA[00000000] OLATB[00000000]
GPIO return code=0, INTFA[00000000] INTFB[00000000] INTCAPA[00111000] INTCAPB[11000000] GPIOA[00111000] GPIOB[00000000] OLATA[00000000] OLATB[00000000]
GPIO return code=0, INTFA[00000000] INTFB[00000000] INTCAPA[00111000] INTCAPB[11000000] GPIOA[00111000] GPIOB[00000000] OLATA[00000000] OLATB[00000000]
GPIO return code=0, INTFA[00000000] INTFB[00000000] INTCAPA[00111000] INTCAPB[11000000] GPIOA[00111000] GPIOB[00000000] OLATA[00000000] OLATB[00000000]
GPIO return code=0, INTFA[00000001] INTFB[00000000] INTCAPA[00001001] INTCAPB[11000000] GPIOA[,,,,PA3,,,PA0] GPIOB[,,,,,,,]
GPIO return code=0, INTFA[00001000] INTFB[00000000] INTCAPA[00000001] INTCAPB[11000000] GPIOA[,,,,,,,] GPIOB[,,,,,,,]
GPIO return code=0, INTFA[00000001] INTFB[00000000] INTCAPA[00000001] INTCAPB[11000000] GPIOA[,,,,,,,PA0] GPIOB[,,,,,,,]
GPIO return code=0, INTFA[00000001] INTFB[00000000] INTCAPA[00000000] INTCAPB[11000000] GPIOA[,,,,,,,] GPIOB[,,,,,,,]
GPIO return code=0, INTFA[10000000] INTFB[00000000] INTCAPA[10000000] INTCAPB[11000000] GPIOA[PA7,,,,,,,] GPIOB[,,,,,,,]
GPIO return code=0, INTFA[00001000] INTFB[00000000] INTCAPA[10001000] INTCAPB[11000000] GPIOA[PA7,,,,PA3,,,] GPIOB[,,,,,,,]

*/