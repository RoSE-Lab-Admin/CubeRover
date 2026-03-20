#include "messages.h"

// Send a message to the ROS code
void send_message(MessageCode msg_code, const String &str) {
  // Send the full message in one go!
  Serial.print(GENERAL_MESSAGE);
  Serial.print(" ");
  Serial.print(static_cast<uint8_t>(msg_code)); // Safely casts the strict enum to a number

  // If the message is not empty, then send it
  if (str.length() > 0) {
    Serial.print(" ");
    Serial.print(str);  // Arduino's Serial.print automatically handles strings of any length  
  }

  Serial.print("\n");
}
