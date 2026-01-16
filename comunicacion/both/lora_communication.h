
#ifndef LORA_COMM_H
#define LORA_COMM_H

#define LORA_LOCAL_ADDRESS      0x04
#define LORA_REMOTE_ADDRESS     0x05

#define LORA_FREQUENCY          868E6
#define LORA_BW                 62.5E3
#define LORA_SF                 10
#define LORA_CR                 5
#define LORA_TP                 2
#define LORA_SYNC_WORD          0x12
#define LORA_PREAMBLE_LENGTH    8


// Inicializa LoRa (config estática)
bool lora_init();

// Envía payload crudo
bool lora_send(const uint8_t *data, uint8_t len);


#endif
