# McpProvider
<a href="https://homieiot.github.io/">
  <img src="https://homieiot.github.io/img/works-with-homie.png" alt="works with MQTT Homie">
</a>

Homie V3 ESP8266 program to read inputs from MCP23017 and send MQTT notification of change.

## Description
Devloped to work with OpenHab and communicate the state of wired door/window sensors.  The MCP23017 has 16 pins which this program labels "pin0" thru "pin15".  To handle the actual sensors which are sometimes Normally-Closed and Normally-Open, there is a custom configuration setting which allow the IPOL inversion register of the MCP to invert the signals to match human expectations.  Each pin will emit either OPEN or CLOSED when triggered.

## Components

* ESP8266
* Waveshare MCP23017 module
* Homie V3+

## Homie Config
```
{
  "name": "Wired Alarm System",
  "device_id": "<device-location-name>",
  "device_stats_interval": 900,  
  "wifi": {
    "ssid": "<wifi-host>",
    "password": "<wifi-password>"
  },
  "mqtt": {
    "host": "<mqtt-hostname-or-ip>",
    "port": 1883,
	"base_topic": "sknSensors/",
    "auth": true,
    "username": "<mqtt-username>",
    "password": "<mqtt-password>"
  },
  "ota": {
    "enabled": true
  },
  "settings": {
      "ipolA": 255, 
      "ipolB": 255
  }
}
```

## Homie MQTT Log
```
sknSensors/MCPProviderNode/$state lost
sknSensors/MCPProviderNode/$state init
sknSensors/MCPProviderNode/$homie 3.0.1
sknSensors/MCPProviderNode/$name Hard Wired Alarm Sensors
sknSensors/MCPProviderNode/$mac 38:2B:78:03:93:CB
sknSensors/MCPProviderNode/$localip 10.100.1.165
sknSensors/MCPProviderNode/$nodes wiredMonitor
sknSensors/MCPProviderNode/$stats uptime
sknSensors/MCPProviderNode/$stats/interval 305
sknSensors/MCPProviderNode/$fw/name mcp-module
sknSensors/MCPProviderNode/$fw/version 1.0.0
sknSensors/MCPProviderNode/$fw/checksum a7beff4874a1acc86fd6cf569ebb58fe
sknSensors/MCPProviderNode/$implementation esp8266
sknSensors/MCPProviderNode/$implementation/config {"name":"Hard Wired Alarm Sensors","device_id":"MCPProviderNode","device_stats_interval":300,"wifi":{"ssid":"SFNSS1-24G"},"mqtt":{"host":"openhab.skoona.net","port":1883,"base_topic":"sknSensors/","auth":true},"ota":{"enabled":true},"settings":{"ipolA":255,"ipolB":255,"broadcastInterval":60}}
sknSensors/MCPProviderNode/$implementation/version 3.0.0
sknSensors/MCPProviderNode/$implementation/ota/enabled true
sknSensors/MCPProviderNode/wiredMonitor/$name Wired Sensors
sknSensors/MCPProviderNode/wiredMonitor/$type sensor
sknSensors/MCPProviderNode/wiredMonitor/$properties pin0,pin1,pin2,pin3,pin4,pin5,pin6,pin7,pin8,pin9,pin10,pin11,pin12,pin13,pin14,pin15
sknSensors/MCPProviderNode/wiredMonitor/pin0/$name Pin 0
sknSensors/MCPProviderNode/wiredMonitor/pin0/$datatype enum
sknSensors/MCPProviderNode/wiredMonitor/pin0/$format OPEN,CLOSED
sknSensors/MCPProviderNode/wiredMonitor/pin1/$name Pin 1
sknSensors/MCPProviderNode/wiredMonitor/pin1/$datatype enum
sknSensors/MCPProviderNode/wiredMonitor/pin1/$format OPEN,CLOSED
sknSensors/MCPProviderNode/wiredMonitor/pin2/$name Pin 2
sknSensors/MCPProviderNode/wiredMonitor/pin2/$datatype enum
sknSensors/MCPProviderNode/wiredMonitor/pin2/$format OPEN,CLOSED
sknSensors/MCPProviderNode/wiredMonitor/pin3/$name Pin 3
sknSensors/MCPProviderNode/wiredMonitor/pin3/$datatype enum
sknSensors/MCPProviderNode/wiredMonitor/pin3/$format OPEN,CLOSED
sknSensors/MCPProviderNode/wiredMonitor/pin4/$name Pin 4
sknSensors/MCPProviderNode/wiredMonitor/pin4/$datatype enum
sknSensors/MCPProviderNode/wiredMonitor/pin4/$format OPEN,CLOSED
sknSensors/MCPProviderNode/wiredMonitor/pin5/$name Pin 5
sknSensors/MCPProviderNode/wiredMonitor/pin5/$datatype enum
sknSensors/MCPProviderNode/wiredMonitor/pin5/$format OPEN,CLOSED
sknSensors/MCPProviderNode/wiredMonitor/pin6/$name Pin 6
sknSensors/MCPProviderNode/wiredMonitor/pin6/$datatype enum
sknSensors/MCPProviderNode/wiredMonitor/pin6/$format OPEN,CLOSED
sknSensors/MCPProviderNode/wiredMonitor/pin7/$name Pin 7
sknSensors/MCPProviderNode/wiredMonitor/pin7/$datatype enum
sknSensors/MCPProviderNode/wiredMonitor/pin7/$format OPEN,CLOSED
sknSensors/MCPProviderNode/wiredMonitor/pin8/$name Pin 8
sknSensors/MCPProviderNode/wiredMonitor/pin8/$datatype enum
sknSensors/MCPProviderNode/wiredMonitor/pin8/$format OPEN,CLOSED
sknSensors/MCPProviderNode/wiredMonitor/pin9/$name Pin 9
sknSensors/MCPProviderNode/wiredMonitor/pin9/$datatype enum
sknSensors/MCPProviderNode/wiredMonitor/pin9/$format OPEN,CLOSED
sknSensors/MCPProviderNode/wiredMonitor/pin10/$name Pin 10
sknSensors/MCPProviderNode/wiredMonitor/pin10/$datatype enum
sknSensors/MCPProviderNode/wiredMonitor/pin10/$format OPEN,CLOSED
sknSensors/MCPProviderNode/wiredMonitor/pin11/$name Pin 11
sknSensors/MCPProviderNode/wiredMonitor/pin11/$datatype enum
sknSensors/MCPProviderNode/wiredMonitor/pin11/$format OPEN,CLOSED
sknSensors/MCPProviderNode/wiredMonitor/pin12/$name Pin 12
sknSensors/MCPProviderNode/wiredMonitor/pin12/$datatype enum
sknSensors/MCPProviderNode/wiredMonitor/pin12/$format OPEN,CLOSED
sknSensors/MCPProviderNode/wiredMonitor/pin13/$name Pin 13
sknSensors/MCPProviderNode/wiredMonitor/pin13/$datatype enum
sknSensors/MCPProviderNode/wiredMonitor/pin13/$format OPEN,CLOSED
sknSensors/MCPProviderNode/wiredMonitor/pin14/$name Pin 14
sknSensors/MCPProviderNode/wiredMonitor/pin14/$datatype enum
sknSensors/MCPProviderNode/wiredMonitor/pin14/$format OPEN,CLOSED
sknSensors/MCPProviderNode/wiredMonitor/pin15/$name Pin 15
sknSensors/MCPProviderNode/wiredMonitor/pin15/$datatype enum
sknSensors/MCPProviderNode/wiredMonitor/pin15/$format OPEN,CLOSED
sknSensors/MCPProviderNode/$state ready
sknSensors/MCPProviderNode/$stats/interval 305
sknSensors/MCPProviderNode/$stats/signal 62
sknSensors/MCPProviderNode/$stats/uptime 4
```
