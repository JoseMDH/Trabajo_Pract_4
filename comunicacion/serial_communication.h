#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "serial_protocol.h"
#include "serial_framing.h"

// =====================
// Communication API
// =====================

// Send a Message over Serial1
bool comm_send_message(const Message *msg, FrameTxContext *tx);

// Poll Serial1, return true if a complete Message was received
bool comm_poll_message(FrameRxContext *rx, Message *out_msg);

#endif // COMMUNICATION_H
