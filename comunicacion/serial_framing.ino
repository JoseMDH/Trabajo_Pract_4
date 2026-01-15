#include "serial_framing.h"

bool message_to_frame(const Message *msg, FrameTxContext *tx)
{
  uint16_t msg_len = MESSAGE_HEADER_SIZE + msg->plen;

  if (msg_len > MAX_MESSAGE_SIZE)
    return false;

  tx->buffer[0] = FRAME_START_BYTE;
  tx->buffer[1] = (uint8_t)msg_len;

  uint8_t *p = &tx->buffer[2];

  p[0] = msg->flags;
  p[1] = msg->cmd;
  p[2] = msg->dir;
  p[3] = msg->status;
  p[4] = msg->plen;

  for (uint8_t i = 0; i < msg->plen; ++i) {
    p[5 + i] = msg->payload[i];
  }

  tx->len = 2 + msg_len;
  return true;
}

bool frame_rx_push_byte(FrameRxContext *rx, uint8_t byte)
{
  if (!rx->in_frame) {
    if (byte == FRAME_START_BYTE) {
      rx->in_frame = true;
      rx->index = 0;
      rx->expected_len = 0;
    }
    return false;
  }

  // first byte after start = LEN
  if (rx->index == 0) {
    rx->expected_len = byte;

    if (rx->expected_len == 0 ||
        rx->expected_len > MAX_MESSAGE_SIZE) {
      rx->in_frame = false;
      return false;
    }

    rx->index++;
    return false;
  }

  rx->buffer[rx->index - 1] = byte;
  rx->index++;

  if (rx->index - 1 == rx->expected_len) {
    rx->in_frame = false;
    return true;
  }

  return false;
}

bool frame_to_message(const uint8_t *payload,
                      uint16_t payload_len,
                      Message *out_msg)
{
  if (payload_len < MESSAGE_HEADER_SIZE)
    return false;

  out_msg->flags  = payload[0];
  out_msg->cmd    = payload[1];
  out_msg->dir    = payload[2];
  out_msg->status = payload[3];
  out_msg->plen   = payload[4];

  if (out_msg->plen != payload_len - MESSAGE_HEADER_SIZE)
    return false;

  for (uint8_t i = 0; i < out_msg->plen; ++i) {
    out_msg->payload[i] = payload[5 + i];
  }

  return true;
}
