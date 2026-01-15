#ifndef FRAMING_H
#define FRAMING_H

#include "serial_protocol.h"

// =====================
// Framing constants
// =====================

#define FRAME_START_BYTE     0x00

#define MAX_PAYLOAD_SIZE     8
#define MESSAGE_HEADER_SIZE  5
#define MAX_MESSAGE_SIZE     (MESSAGE_HEADER_SIZE + MAX_PAYLOAD_SIZE)
#define MAX_FRAME_SIZE       (2 + MAX_MESSAGE_SIZE) // start + len + message

// =====================
// RX / TX contexts
// =====================

typedef struct {
  uint8_t  buffer[MAX_FRAME_SIZE];
  uint16_t index;
  uint16_t expected_len;
  bool     in_frame;
} FrameRxContext;

typedef struct {
  uint8_t  buffer[MAX_FRAME_SIZE];
  uint16_t len;
} FrameTxContext;

// =====================
// Framing API
// =====================

bool message_to_frame(const Message *msg, FrameTxContext *tx);
bool frame_to_message(const uint8_t *payload,
                      uint16_t payload_len,
                      Message *out_msg);

bool frame_rx_push_byte(FrameRxContext *rx, uint8_t byte);

#endif // FRAMING_H
