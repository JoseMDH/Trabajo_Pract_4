#!/usr/bin/env python3
"""
MQTT-LoRa Bridge para Raspberry Pi
===================================
Este script actúa como puente entre MQTT y LoRa a través de comunicación
serial con un Arduino MKR WAN 1310.

Funcionalidades:
- Recibe mensajes del Arduino (datos LoRa) y los publica en topics MQTT
- Se suscribe a topics MQTT y envía comandos al Arduino para reenviar vía LoRa

Protocolo Serial:
- STX (0x02) | tipo (1 byte) | topic_len | topic | payload_len | payload | ETX (0x03)
- Tipo: 'R' = Recibido de LoRa, 'T' = Transmitir por LoRa
"""

import serial
import threading
import time
import json
import struct
import paho.mqtt.client as mqtt
from dataclasses import dataclass
from typing import Optional, Callable
from enum import IntEnum

# =====================
# Configuración
# =====================
SERIAL_PORT = "/dev/ttyACM0"  # Puerto serie del Arduino (ajustar según sistema)
SERIAL_BAUD = 115200

MQTT_BROKER = "localhost"
MQTT_PORT = 1883

# Topics MQTT
TOPIC_SENSOR_LUZ = "sensores/luz"           # Datos del sensor de luz
TOPIC_SENSOR_PUERTA = "sensores/puerta"     # Estado de la puerta
TOPIC_LORA_RX = "lora/rx"                   # Mensajes recibidos por LoRa (raw)
TOPIC_LORA_TX = "lora/tx"                   # Mensajes a enviar por LoRa
TOPIC_ACTUADOR = "actuador/comando"         # Comandos para actuadores remotos

# Topics a los que suscribirse para enviar por LoRa
TOPICS_TO_LORA = [
    TOPIC_LORA_TX,
    TOPIC_ACTUADOR
]

# =====================
# Constantes del protocolo serial
# =====================
STX = 0x02
ETX = 0x03

class MsgType(IntEnum):
    LORA_RX = ord('R')      # Mensaje recibido de LoRa
    LORA_TX = ord('T')      # Mensaje a transmitir por LoRa
    ACK = ord('A')          # Acknowledgment
    NACK = ord('N')         # Negative acknowledgment
    STATUS = ord('S')       # Estado del sistema


@dataclass
class LoRaMessage:
    """Estructura de un mensaje LoRa"""
    sender: int
    rssi: int
    snr: float
    topic: str
    payload: bytes
    
    def to_dict(self):
        return {
            "sender": f"0x{self.sender:02X}",
            "rssi": self.rssi,
            "snr": self.snr,
            "topic": self.topic,
            "payload": self.payload.decode('utf-8', errors='replace')
        }


class SerialProtocol:
    """Maneja la comunicación serial con el Arduino"""
    
    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.rx_buffer = bytearray()
        self.running = False
        self.rx_callback: Optional[Callable] = None
        self.rx_thread: Optional[threading.Thread] = None
        
    def connect(self) -> bool:
        """Conecta al puerto serie"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1
            )
            time.sleep(2)  # Esperar reset del Arduino
            print(f"[Serial] Conectado a {self.port}")
            return True
        except serial.SerialException as e:
            print(f"[Serial] Error de conexión: {e}")
            return False
    
    def disconnect(self):
        """Desconecta del puerto serie"""
        self.running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=2)
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("[Serial] Desconectado")
    
    def start_receiving(self, callback: Callable):
        """Inicia el hilo de recepción"""
        self.rx_callback = callback
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
    
    def _rx_loop(self):
        """Bucle de recepción de datos"""
        while self.running:
            try:
                if self.serial and self.serial.in_waiting:
                    data = self.serial.read(self.serial.in_waiting)
                    self._process_rx_data(data)
            except serial.SerialException as e:
                print(f"[Serial] Error de lectura: {e}")
                time.sleep(1)
            time.sleep(0.01)
    
    def _process_rx_data(self, data: bytes):
        """Procesa datos recibidos y extrae mensajes completos"""
        self.rx_buffer.extend(data)
        
        while True:
            # Buscar STX
            try:
                start_idx = self.rx_buffer.index(STX)
                self.rx_buffer = self.rx_buffer[start_idx:]
            except ValueError:
                self.rx_buffer.clear()
                return
            
            # Verificar longitud mínima: STX + tipo + topic_len + payload_len + ETX
            if len(self.rx_buffer) < 5:
                return
            
            msg_type = self.rx_buffer[1]
            topic_len = self.rx_buffer[2]
            
            if len(self.rx_buffer) < 4 + topic_len:
                return
            
            topic = self.rx_buffer[3:3+topic_len].decode('utf-8', errors='replace')
            payload_len = self.rx_buffer[3+topic_len]
            
            total_len = 5 + topic_len + payload_len  # STX + tipo + topic_len + topic + payload_len + payload + ETX
            
            if len(self.rx_buffer) < total_len:
                return
            
            # Verificar ETX
            if self.rx_buffer[total_len - 1] != ETX:
                # Mensaje corrupto, descartar STX y seguir buscando
                self.rx_buffer = self.rx_buffer[1:]
                continue
            
            # Extraer payload
            payload_start = 4 + topic_len
            payload = bytes(self.rx_buffer[payload_start:payload_start + payload_len])
            
            # Extraer metadatos LoRa si están presentes (sender, rssi, snr)
            # El formato extendido incluye: sender(1) + rssi(1) + snr(1) antes del payload real
            lora_msg = None
            if msg_type == MsgType.LORA_RX and payload_len >= 3:
                sender = payload[0]
                rssi = -int(payload[1])  # RSSI viene como valor positivo
                snr = payload[2] - 128   # SNR viene con offset de 128
                actual_payload = payload[3:]
                lora_msg = LoRaMessage(
                    sender=sender,
                    rssi=rssi,
                    snr=snr,
                    topic=topic,
                    payload=actual_payload
                )
            
            # Notificar callback
            if self.rx_callback:
                self.rx_callback(msg_type, topic, payload, lora_msg)
            
            # Eliminar mensaje procesado del buffer
            self.rx_buffer = self.rx_buffer[total_len:]
    
    def send_message(self, msg_type: int, topic: str, payload: bytes) -> bool:
        """Envía un mensaje por el puerto serie"""
        if not self.serial or not self.serial.is_open:
            return False
        
        try:
            topic_bytes = topic.encode('utf-8')
            frame = bytearray()
            frame.append(STX)
            frame.append(msg_type)
            frame.append(len(topic_bytes))
            frame.extend(topic_bytes)
            frame.append(len(payload))
            frame.extend(payload)
            frame.append(ETX)
            
            self.serial.write(frame)
            self.serial.flush()
            print(f"[Serial] Enviado: tipo={chr(msg_type)}, topic={topic}, payload={payload.hex()}")
            return True
        except serial.SerialException as e:
            print(f"[Serial] Error de envío: {e}")
            return False


class MQTTLoRaBridge:
    """Puente entre MQTT y LoRa"""
    
    def __init__(self):
        self.serial_proto = SerialProtocol(SERIAL_PORT, SERIAL_BAUD)
        self.mqtt_client = mqtt.Client()
        self.mqtt_connected = False
        
    def start(self):
        """Inicia el bridge"""
        # Configurar callbacks MQTT
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_client.on_message = self._on_mqtt_message
        
        # Conectar al broker MQTT
        try:
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            print(f"[MQTT] Error de conexión: {e}")
            return False
        
        # Conectar al puerto serie
        if not self.serial_proto.connect():
            print("[Bridge] No se pudo conectar al Arduino")
            return False
        
        # Iniciar recepción serie
        self.serial_proto.start_receiving(self._on_serial_message)
        
        print("[Bridge] Iniciado correctamente")
        return True
    
    def stop(self):
        """Detiene el bridge"""
        self.serial_proto.disconnect()
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        print("[Bridge] Detenido")
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback cuando se conecta al broker MQTT"""
        if rc == 0:
            print(f"[MQTT] Conectado al broker {MQTT_BROKER}")
            self.mqtt_connected = True
            # Suscribirse a topics para enviar por LoRa
            for topic in TOPICS_TO_LORA:
                client.subscribe(topic)
                print(f"[MQTT] Suscrito a: {topic}")
        else:
            print(f"[MQTT] Error de conexión, código: {rc}")
    
    def _on_mqtt_disconnect(self, client, userdata, rc):
        """Callback cuando se desconecta del broker MQTT"""
        print(f"[MQTT] Desconectado, código: {rc}")
        self.mqtt_connected = False
    
    def _on_mqtt_message(self, client, userdata, msg):
        """Callback cuando se recibe un mensaje MQTT"""
        topic = msg.topic
        payload = msg.payload
        
        print(f"[MQTT] Recibido: {topic} -> {payload.decode('utf-8', errors='replace')}")
        
        # Enviar por LoRa a través del Arduino
        self.serial_proto.send_message(MsgType.LORA_TX, topic, payload)
    
    def _on_serial_message(self, msg_type: int, topic: str, payload: bytes, lora_msg: Optional[LoRaMessage]):
        """Callback cuando se recibe un mensaje del Arduino"""
        print(f"[Serial] Recibido: tipo={chr(msg_type)}, topic={topic}")
        
        if msg_type == MsgType.LORA_RX:
            if lora_msg:
                # Publicar en MQTT con metadatos
                mqtt_payload = json.dumps(lora_msg.to_dict())
                self.mqtt_client.publish(TOPIC_LORA_RX, mqtt_payload)
                print(f"[MQTT] Publicado en {TOPIC_LORA_RX}: {mqtt_payload}")
                
                # También publicar en el topic específico del sensor
                if topic:
                    self.mqtt_client.publish(topic, lora_msg.payload)
                    print(f"[MQTT] Publicado en {topic}: {lora_msg.payload}")
            else:
                # Sin metadatos LoRa, publicar payload directamente
                self.mqtt_client.publish(topic, payload)
        
        elif msg_type == MsgType.ACK:
            print("[Serial] ACK recibido del Arduino")
        
        elif msg_type == MsgType.NACK:
            print("[Serial] NACK recibido del Arduino")
        
        elif msg_type == MsgType.STATUS:
            print(f"[Serial] Estado: {payload.decode('utf-8', errors='replace')}")


def main():
    """Función principal"""
    bridge = MQTTLoRaBridge()
    
    if not bridge.start():
        print("Error al iniciar el bridge")
        return
    
    print("\n" + "="*50)
    print("MQTT-LoRa Bridge activo")
    print("="*50)
    print(f"Puerto serie: {SERIAL_PORT}")
    print(f"Broker MQTT: {MQTT_BROKER}:{MQTT_PORT}")
    print("Presiona Ctrl+C para salir\n")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nDeteniendo bridge...")
        bridge.stop()


if __name__ == "__main__":
    main()
