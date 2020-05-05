

// MCP23017 Example: Slow key press reaction.
//
// Toggle LEDs and detect keypress.
//
// Example code showing slow reaction of 'button'
// LED to keypress. Leading into why interrupts
// are useful (See next example).
//
// Copyright : John Main
// Free for non commercial use.

#include <Wire.h>
#include <Adafruit_MCP23017.h>

#define MCP_LED1 0
#define MCP_INPUTPIN 4
#define MCP_LEDTOG1 1
#define MCP_LEDTOG2 2

Adafruit_MCP23017 mcp;

void setup() {
  mcp.begin();      // Default device address 0

  mcp.pinMode(MCP_LEDTOG1, OUTPUT);  // Toggle LED 1
  mcp.pinMode(MCP_LEDTOG2, OUTPUT);  // Toggle LED 2

  mcp.pinMode(MCP_LED1, OUTPUT);     // LED output
  mcp.digitalWrite(MCP_LED1, HIGH);

  mcp.pinMode(MCP_INPUTPIN, INPUT);  // Button i/p to GND
  mcp.pullUp(MCP_INPUTPIN, HIGH);    // Puled high to ~100k
}

// Alternate LEDTOG1 and LEDTOG2.
// Transfer pin input to LED1.
void loop() {

  delay(300);

  mcp.digitalWrite(MCP_LEDTOG1, HIGH);
  mcp.digitalWrite(MCP_LEDTOG2, LOW);

  delay(300);

  mcp.digitalWrite(MCP_LEDTOG1, LOW);
  mcp.digitalWrite(MCP_LEDTOG2, HIGH);

  // Transfer input pin state to LED1
  if (mcp.digitalRead(MCP_INPUTPIN)) {
    mcp.digitalWrite(MCP_LED1, HIGH);
  } else {
    mcp.digitalWrite(MCP_LED1, LOW);
  }

}
