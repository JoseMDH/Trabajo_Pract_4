#include <SPI.h>
#include <LoRa.h>

#include "lora_communication.h"

// =====================
// RX state
// =====================
static volatile bool rx_available = false;
static LoRaPacket rx_pkt;

// =====================
// RX ISR
// =====================
static void onReceive(int packetSize)
{
  if (packetSize <= 0) return;

  uint8_t dst = LoRa.read();
  uint8_t src = LoRa.read();
  uint8_t len = LoRa.read();

  if (dst != LORA_LOCAL_ADDRESS) {
    while (LoRa.available()) LoRa.read();
    return;
  }

  rx_pkt.dst = dst;
  rx_pkt.src = src;
  rx_pkt.len = 0;

  while (LoRa.available() && rx_pkt.len < LORA_MAX_PAYLOAD) {
    rx_pkt.payload[rx_pkt.len++] = LoRa.read();
  }

  rx_available = true;
}

// =====================
// Init
// =====================
bool lora_comm_init(void)
{
  if (!LoRa.begin(LORA_FREQUENCY))
    return false;

  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);

  LoRa.onReceive(onReceive);
  LoRa.receive();

  return true;
}

// =====================
// TX
// =====================
bool lora_comm_send(const uint8_t *data, uint8_t len)
{
  if (!data || len == 0 || len > LORA_MAX_PAYLOAD)
    return false;

  LoRa.beginPacket();
  LoRa.write(LORA_REMOTE_ADDRESS);
  LoRa.write(LORA_LOCAL_ADDRESS);
  LoRa.write(len);
  LoRa.write(data, len);
  LoRa.endPacket();   // bloqueante

  LoRa.receive();
  return true;
}

// =====================
// RX poll
// =====================
bool lora_comm_poll(LoRaPacket *out)
{
  if (!rx_available || !out)
    return false;

  noInterrupts();
  *out = rx_pkt;
  rx_available = false;
  interrupts();

  return true;
}



// ejemplos

/*
void setup() {
  Serial.begin(115200);

  if (!lora_comm_init()) {
    Serial.println("LoRa init failed");
    while (1);
  }
}

*/

/*
void loop() {
  static uint32_t t0 = 0;

  // TX
  if (millis() - t0 > 2000) {
    uint8_t p[3] = { 0x01, 0x02, 0x03 };
    lora_comm_send(p, 3);
    t0 = millis();
  }

  // RX
  LoRaPacket pkt;
  if (lora_comm_poll(&pkt)) {
    Serial.print("RX from 0x");
    Serial.print(pkt.src, HEX);
    Serial.print(" RSSI=");
    Serial.print(pkt.rssi);
    Serial.print(" SNR=");
    Serial.println(pkt.snr);
  }
}

*/
