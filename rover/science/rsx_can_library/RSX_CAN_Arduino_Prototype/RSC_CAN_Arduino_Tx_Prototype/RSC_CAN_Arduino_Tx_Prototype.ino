#include <SPI.h>
#include <mcp2515.h>

// PINOUT to Arduino Uno
// SCK to Pin 13
// SI to Pin 11
// SO to Pin 12 
// CS to 10 

MCP2515 mcp2515(10);   // CS pin

void setup() {
  Serial.begin(115200);

  SPI.begin();

  // Reset MCP2515
  mcp2515.reset();

  // Set CAN speed (must match your bus!)
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);

  // Switch to normal mode
  mcp2515.setNormalMode();

  Serial.println("MCP2515 init OK");
}

void loop() {
  struct can_frame frame;

  frame.can_id  = 0x123;   // CAN ID
  frame.can_dlc = 2;       // Data length
  frame.data[0] = 0xAA;    // payload
  frame.data[1] = 0x55;

  mcp2515.sendMessage(&frame);

  Serial.println("Message sent");
  delay(1000);
}
