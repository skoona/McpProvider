# McpProvider
<a href="https://homieiot.github.io/">
  <img src="https://homieiot.github.io/img/works-with-homie.png" alt="works with MQTT Homie">
</a>

Homie V3 ESP8266 program to read inputs from MCP23017 and send MQTT notification of change.

## Description
Devloped to work with OpenHab and communicate the state of wired door/window sensors.  The MCP23017 has 16 pins which this program labels "pin0" thru "pin15".  To handle the actual sensors which are sometimes Normally-Closed and Normally-Open, there is a custom configuration setting which allow the IPOL inversion register of the MCP to invert the signals to match human expectations.  Each pin will emit either OPEN or CLOSED when triggered.

## Components

* D1-Mini ESP8266
* Waveshare MCP23017 module
* Homie V3+

