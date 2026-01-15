
#ifndef LORA_COMM_H
#define LORA_COMM_H

// =====================
// LORA DIR
// =====================
#define LORA_LOCAL_ADDRESS      0x05
#define LORA_REMOTE_ADDRESS     0x06

// =====================
// LORA CONF (estáticos)
// =====================
#define LORA_FREQUENCY          868E6
#define LORA_SYNC_WORD          0x12
#define LORA_PREAMBLE_LENGTH    8

// =====================
// Payload
// =====================
#define LORA_MAX_PAYLOAD        64



// =====================
// RX packet descriptor
// =====================
typedef struct {
  uint8_t  src;
  uint8_t  dst;
  uint8_t  len;
  uint8_t  payload[LORA_MAX_PAYLOAD];
} LoRaPacket;

// =====================
// API
// =====================

// Inicializa LoRa (config estática)
bool lora_comm_init(void);

// Envía payload crudo
bool lora_comm_send(const uint8_t *data, uint8_t len);

// Poll: devuelve true si hay paquete completo
bool lora_comm_poll(LoRaPacket *out);

#endif
