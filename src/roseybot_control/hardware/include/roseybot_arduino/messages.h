#ifndef MESSAGES
#define MESSAGES

#define GENERAL_MESSAGE 'm'
#define TELEMETRY_MESSAGE 't'

enum class ErrorCode : uint8_t {
  OK,
  CHECK_ENCODER,
  CHECK_VELOCITY
};

// Helper function to map the enum back to a readable string
inline const char* errorCodeToString(int code) {
  // Cast the integer back into our enum class type
  switch (static_cast<ErrorCode>(code)) {
    case ErrorCode::OK: return "OK";
    case ErrorCode::CHECK_ENCODER: return "CHECK_ENCODER";
    case ErrorCode::CHECK_VELOCITY: return "CHECK_VELOCITY";
    default: return "UNKNOWN_ERROR";
  }
}

#endif