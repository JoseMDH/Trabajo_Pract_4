#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

#include "lora_communication.h"

// =====================
// Init
// =====================
bool lora_init() {
  if (!LoRa.begin(LORA_FREQUENCY))
    return false;
  LoRa.setSignalBandwidth(long(LORA_BW)); 
  LoRa.setSpreadingFactor(LORA_SF); 
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setTxPower(LORA_TP, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);
  LoRa.receive();
  return true;
}

// =====================
// TX
// =====================
bool lora_send(const uint8_t *data, uint8_t len)
{
  if (!data || len == 0 )
    return false;
  LoRa.beginPacket();
  LoRa.write(LORA_REMOTE_ADDRESS);
  LoRa.write(LORA_LOCAL_ADDRESS);
  LoRa.write(len);
  LoRa.write(data, (size_t)len);
  LoRa.endPacket();   // bloqueante, SINO (true)
  LoRa.receive();
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
