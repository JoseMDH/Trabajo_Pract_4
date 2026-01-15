#ifndef PROTOCOL_H
#define PROTOCOL_H


// =====================
// Message (protocol core)
// =====================

typedef struct {
  uint8_t flags;     // MSG_FLAG_*
  uint8_t cmd;       // CommandId
  uint8_t dir;       // SRF02 I2C address (0xE2, 0xF2)
  uint8_t status;    // ErrorCode (responses only)
  uint8_t plen;      // payload length
  uint8_t payload[]; // variable-length payload
} Message;

// Message flags
#define MSG_FLAG_RESPONSE  (1 << 0)
#define MSG_FLAG_EVENT     (1 << 1)

// =====================
// Commands
// =====================

typedef enum {
  CMD_HELP = 0,         // terminal-only
  CMD_RANGE_ONCE,       // us <srf02> one-shot
  CMD_STREAM_ON,        // us <srf02> on <period_ms>
  CMD_STREAM_OFF,       // us <srf02> off
  CMD_SET_UNIT,         // us <srf02> unit {inc|cm|ms}
  CMD_SET_DELAY,        // us <srf02> delay <ms>
  CMD_GET_STATUS,       // us <srf02> status
  CMD_LIST_SENSORS,     // us
} CommandId;

// HELP is a protocol-level message (used only by supervisor)
extern const Message HELP_MESSAGE;

// =====================
// Errors
// =====================

typedef enum {
  ERR_OK = 0,
  ERR_BAD_CMD,
  ERR_BAD_DIR,
  ERR_BAD_PARAM,
  ERR_I2C_NACK,
  ERR_I2C_TIMEOUT,
  ERR_SENSOR_BUSY,
  ERR_INTERNAL,
  ERR_TIMEOUT,
} ErrorCode;

// =====================
// Units (SRF02 commands)
// =====================

typedef enum {
  UNIT_INCHES = 0x50,   // 80
  UNIT_CM     = 0x51,   // 81
  UNIT_USEC   = 0x52,   // 82
} Unit;

typedef struct {
  uint8_t  dir;
  Unit     unit;
  uint16_t min_delay_ms;
  bool     periodic_enabled;
  uint16_t period_ms;
} SensorStatus;

#endif // PROTOCOL_H
