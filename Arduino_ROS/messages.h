#ifndef MESSAGES
#define MESSAGES

#include <Arduino.h>

// define character types for different messages
#define GENERAL_MESSAGE 'm' // Format: <error_code> <message> - A general message. Max length: 256 bytes.
#define TELEMETRY_MESSAGE 't' // Format: <telemetry data stream> - The data stream is determined by the get_telemetry() function

enum class ErrorCode : uint8_t {
  OK,
  CHECK_ENCODER,
  CHECK_VELOCITY
};

void send_message(ErrorCode err, const String &str);

#endif