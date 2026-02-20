
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

  Serial.println("MCP2515 init OK Yayyyy :)");
}

void loop() {
  struct can_frame can_msg;

  MCP2515::ERROR res = mcp2515.readMessage(&can_msg);

  if (res != MCP2515::ERROR_NOMSG){
    Serial.print("Error Message: ");
    Serial.println(res);
    
    Serial.print("DLC: ");
    Serial.println(can_msg.can_dlc);
    
    Serial.print("CAN ID: ");
    Serial.println(can_msg.can_id, HEX);
    
    Serial.print("CAN Message: ");
    for (int i=0; i<8; i++){
      Serial.print(int(can_msg.data[i]), HEX);
      Serial.print(",");
    }
    Serial.println("");
    Serial.println("--------------------");
  }

//  Serial.println("--");
}
