#include "serial_communication.h"

bool comm_send_message(const Message *msg, FrameTxContext *tx)
{
  if (!message_to_frame(msg, tx))
    return false;

  for (uint16_t i = 0; i < tx->len; ++i) {
    Serial1.write(tx->buffer[i]);
  }

  return true;
}

bool comm_poll_message(FrameRxContext *rx, Message *out_msg)
{
  while (Serial1.available()) {
    uint8_t b = Serial1.read();

    if (frame_rx_push_byte(rx, b)) {
      return frame_to_message(rx->buffer,
                              rx->expected_len,
                              out_msg);
    }
  }
  return false;
}
