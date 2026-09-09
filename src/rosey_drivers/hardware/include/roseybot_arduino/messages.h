#ifndef MESSAGES
#define MESSAGES

#include <cstdint>

#define GENERAL_MESSAGE 'm'
#define TELEMETRY_MESSAGE 't'

enum class MessageCode : uint8_t {
  OK,
  CHECK_ENCODER,
  CHECK_VELOCITY
};

// Helper function to map the enum back to a readable string
inline const char* messageCodeToString(int code) {
  // Cast the integer back into our enum class type
  switch (static_cast<MessageCode>(code)) {
    case MessageCode::OK: return "OK";
    case MessageCode::CHECK_ENCODER: return "CHECK_ENCODER";
    case MessageCode::CHECK_VELOCITY: return "CHECK_VELOCITY";
    default: return "UNKNOWN_MESSAGE";
  }
}

#endif