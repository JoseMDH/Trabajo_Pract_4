# Sistema de Domótica con LoRa y MQTT

**Asignatura:** Internet de las cosas (GII-IC)  
**Autores:** José Manuel Díaz Hernández, Nicolás Rey Alonso, Santiago, Alberto
**Fecha:** Enero 2026

---

##  Descripción General

Sistema de domótica distribuido que permite el control automático de iluminación y puertas basándose en sensores de luz y proximidad. La comunicación entre dispositivos se realiza mediante **LoRa** (largo alcance, bajo consumo) y la integración con servicios externos mediante **MQTT**.

### Arquitectura del Sistema

```
┌─────────────────┐     Serial      ┌─────────────────┐
│     ESCLAVO     │◄───────────────►│     MAESTRO     │
│  (Arduino UNO)  │                 │ (MKR WAN 1310)  │
│  - Sensor Luz   │                 │  - Supervisor   │
│  - SRF01/SRF02  │                 │  - LoRa TX      │
└─────────────────┘                 └────────┬────────┘
                                             │ LoRa 868MHz
                                             ▼
┌─────────────────┐     Serial      ┌─────────────────┐
│   RASPBERRY PI  │◄───────────────►│    GATEWAY      │
│  - MQTT Broker  │    (GPIO UART)  │ (MKR WAN 1310)  │
│  - Bridge       │    MQTT         │  - LoRa RX/TX   │
└────────┬────────┘                 └────────┬────────┘
         │ MQTT                              │ LoRa 868MHz
         ▼                                   ▼
┌─────────────────┐                 ┌─────────────────┐
│  Aplicaciones   │                 │    ACTUADOR     │
│  - Dashboard    │                 │  (Arduino UNO)  │
│  - Node-RED     │                 │  - LED          │
│  - Home Assist  │                 │  - Servo        │
└─────────────────┘                 └─────────────────┘
```

---

##  Estructura del Proyecto

```
Trabajo_Pract_4/
├── sensores/
│   ├── Maestro/           # Supervisor con LoRa (MKR WAN 1310)
│   │   └── Maestro.ino
│   ├── Esclavo/           # Nodo sensor (Arduino MKR WAN)
│   │   └── Esclavo.ino
│   └── README.md
│
├── Receptor/
│   ├── Gateway/           # Gateway LoRa-Serial (MKR WAN 1310)
│   │   ├── Gateway.ino
│   │   └── BQ24195L_PMIC.ino
│   └── broker/            # Bridge MQTT-LoRa (Raspberry Pi)
│       ├── broker.py
│       ├── mqtt_lora_bridge.py
│       ├── requirements.txt
│       └── README.md
│
├── actuador/              # Actuador con LoRa (Arduino UNO + módulo LoRa)
│   ├── actuador.ino
│   └── README.md
│
├── comunicacion/          # Pruebas de comunicación LoRa
│   ├── both/
│   ├── comunicacion/
│   └── publisher/
│
└── pruebasSensores/       # Sketches de prueba individuales
    ├── Led/
    ├── SensorLuz/
    └── Servo/
```

---

## Componentes de Hardware

### Nodo Sensor (Esclavo + Maestro)

| Componente | Descripción |
|------------|-------------|
| Arduino UNO | Esclavo - lectura de sensores |
| Arduino MKR WAN 1310 | Maestro - comunicación LoRa |
| SRF01 / SRF02 | Sensores ultrasónicos I2C |
| LDR + Resistencia | Sensor de luz (divisor de voltaje) |

### Gateway

| Componente | Descripción |
|------------|-------------|
| Arduino MKR WAN 1310 | Gateway LoRa ↔ Serial |
| Raspberry Pi | Broker MQTT + Bridge |

### Actuador

| Componente | Descripción |
|------------|-------------|
| Arduino UNO | Controlador |
| Módulo LoRa (SX1276) | Recepción de comandos |
| LED | Control de iluminación |
| Servo SG90 | Control de puerta |

---

## ⚡ Configuración LoRa

Todos los dispositivos deben usar la **misma configuración**:

```cpp
Frecuencia:     868 MHz (Europa)
Ancho de banda: 62.5 kHz
Spreading Factor: 10
Coding Rate:    4/5
TX Power:       2 dBm
Sync Word:      0x12
Preamble:       8 símbolos
```

### Direcciones LoRa

| Dispositivo | Dirección |
|-------------|-----------|
| Maestro (Sensores) | `0x04` |
| Gateway | `0x05` |
| Actuador | `0x06` |
| Broadcast | `0xFF` |

---

## 📡 Protocolo de Comunicación

### Formato de Paquete LoRa

```
┌──────────┬──────────┬─────────┬─────────┬──────────┬─────────┐
│ Destino  │ Origen   │ Msg ID  │ Msg ID  │ Longitud │ Payload │
│ (1 byte) │ (1 byte) │ (MSB)   │ (LSB)   │ (1 byte) │ (N bytes)│
└──────────┴──────────┴─────────┴─────────┴──────────┴─────────┘
```

### Payload de Sensores (Maestro → Gateway)

```
┌───────────┬───────────────────────┬─────────┐
│ TopicLen  │ Topic                 │ Valor   │
│ (1 byte)  │ (TopicLen bytes)      │ (1 byte)│
└───────────┴───────────────────────┴─────────┘

Topics:
- "sensor/0" → sensores/puerta (distancia)
- "sensor/1" → sensores/luz
```

### Payload de Actuador (Gateway → Actuador)

```
┌──────────┬─────────┐
│ Tipo     │ Valor   │
│ (1 byte) │ (1 byte)│
└──────────┴─────────┘

Tipo:
- 0 = Luz (valor: 0=apagar, 1=encender)
- 1 = Puerta (valor: 0=cerrar, 1/2/3=abrir)
```

### Protocolo Serial (Gateway ↔ Raspberry Pi)

```
┌─────┬──────┬───────────┬───────┬─────────────┬─────────┬─────┐
│ STX │ Tipo │ TopicLen  │ Topic │ PayloadLen  │ Payload │ ETX │
│0x02 │1 byte│ 1 byte    │N bytes│ 1 byte      │ N bytes │0x03 │
└─────┴──────┴───────────┴───────┴─────────────┴─────────┴─────┘

Tipos:
- 'R' (0x52): Mensaje recibido de LoRa
- 'T' (0x54): Mensaje a transmitir por LoRa
- 'A' (0x41): ACK
- 'N' (0x4E): NACK
- 'S' (0x53): Estado
```

---

## 🚀 Instalación y Configuración

### 1. Esclavo (Arduino UNO)

```bash
# Abrir en Arduino IDE
sensores/Esclavo/Esclavo.ino

# Conexiones:
# - SRF01: I2C (A4=SDA, A5=SCL)
# - SRF02: I2C (A4=SDA, A5=SCL)
# - LDR: A1 (con divisor de voltaje)
# - Serial1: Pin 8 (RX), Pin 9 (TX) → conectar a Maestro
```

### 2. Maestro (MKR WAN 1310)

```bash
# Abrir en Arduino IDE
sensores/Maestro/Maestro.ino

# Librerías necesarias:
# - LoRa (by Sandeep Mistry)
# - Arduino_PMIC

# Conexiones:
# - Serial1: Pin 13 (RX), Pin 14 (TX) → conectar a Esclavo
# - LoRa: Integrado en MKR WAN 1310
```

### 3. Gateway (MKR WAN 1310)

```bash
# Abrir en Arduino IDE
Receptor/Gateway/Gateway.ino

# Conexiones a Raspberry Pi:
# - Pin 13 (RX) → Raspberry GPIO14 (TX)
# - Pin 14 (TX) → Raspberry GPIO15 (RX)
# - GND → GND común
```

### 4. Raspberry Pi

```bash
# Deshabilitar consola serial en raspi-config
sudo raspi-config
# → Interface Options → Serial Port → No (login shell) → Yes (hardware)

# Instalar dependencias
cd Receptor/broker
pip3 install -r requirements.txt

# Instalar y configurar Mosquitto (broker MQTT)
sudo apt install mosquitto mosquitto-clients
sudo systemctl enable mosquitto
sudo systemctl start mosquitto

# Ejecutar el bridge
python3 mqtt_lora_bridge.py
```

### 5. Actuador (Arduino UNO + Módulo LoRa)

```bash
# Abrir en Arduino IDE
actuador/actuador.ino

# Conexiones módulo LoRa (SX1276):
# - NSS  → Pin 10
# - MOSI → Pin 11
# - MISO → Pin 12
# - SCK  → Pin 13
# - RST  → Pin 9
# - DIO0 → Pin 2

# Conexiones actuadores:
# - Servo → Pin 7
# - LED   → Pin 6
```

---

## 🎮 Uso del Sistema

### Comandos del Maestro (Monitor Serial)

```
help                    → Muestra ayuda
us                      → Lista sensores disponibles
us <id> one-shot        → Lectura única del sensor
us <id> on <period_ms>  → Activar lectura periódica
us <id> off             → Desactivar lectura periódica
us <id> unit {cm|inc|ms}→ Cambiar unidad de medida
us <id> delay <ms>      → Establecer retardo entre lecturas
us <id> status          → Ver configuración del sensor
```

### Topics MQTT

| Topic | Dirección | Descripción |
|-------|-----------|-------------|
| `sensores/luz` | Entrada | Estado de luz (0=iluminado, 1=oscuro) |
| `sensores/puerta` | Entrada | Estado proximidad (0=libre, 1=objeto cerca) |
| `lora/rx` | Entrada | Mensajes LoRa raw (JSON) |
| `lora/tx` | Salida | Enviar mensaje LoRa genérico |
| `actuador/comando` | Salida | Comandos para actuador |

### Probar con mosquitto_pub/sub

```bash
# Suscribirse a todos los mensajes de sensores
mosquitto_sub -h localhost -t "sensores/#" -v

# Encender luz manualmente
mosquitto_pub -h localhost -t "sensores/luz" -m "1"

# Abrir puerta manualmente
mosquitto_pub -h localhost -t "sensores/puerta" -m "1"

# Cerrar puerta
mosquitto_pub -h localhost -t "sensores/puerta" -m "0"
```

---

## 📊 Umbrales y Lógica

### Sensor de Luz (LDR)

| Valor LDR | Estado | Acción |
|-----------|--------|--------|
| < 500 | Oscuro (1) | Encender LED |
| ≥ 500 | Iluminado (0) | Apagar LED |

### Sensor de Distancia (Ultrasonidos)

| Distancia | Estado | Acción |
|-----------|--------|--------|
| < 100 cm | Objeto cerca (1) | Abrir puerta |
| ≥ 100 cm | Libre (0) | Cerrar puerta |

---

## 🔍 Depuración

### Verificar comunicación LoRa

En el **Gateway** (monitor serial USB a 115200 baud):
```
LoRa RX: topic=sensor/1 -> sensores/luz len=4
```

En el **Actuador** (monitor serial a 9600 baud):
```
========== PAQUETE RECIBIDO ==========
Tamaño del paquete: 7
RSSI: -45 dBm
SNR: 9.5 dB
Destinatario: 0x06
Remitente: 0x05
-> Tipo=0 (luz), Valor=1
Luz -> ENCENDER (1)
=======================================
```

### Verificar MQTT

```bash
# Ver todos los mensajes MQTT
mosquitto_sub -h localhost -t "#" -v
```

---

## ⚠️ Solución de Problemas

| Problema | Solución |
|----------|----------|
| LoRa no inicializa | Verificar conexiones SPI del módulo |
| No recibe mensajes | Comprobar que todos usen misma config LoRa |
| Actuador no responde | Verificar dirección destino (0x06) y emisor (0x05) |
| Bridge no conecta | Verificar puerto serial `/dev/serial0` |
| MQTT no publica | Comprobar que Mosquitto está corriendo |

---

## 📄 Licencia

Proyecto académico - Universidad de Las Palmas de Gran Canaria (ULPGC)

---

## 📚 Referencias

- [Librería LoRa Arduino](https://github.com/sandeepmistry/arduino-LoRa)
- [Arduino MKR WAN 1310](https://docs.arduino.cc/hardware/mkr-wan-1310)
- [Paho MQTT Python](https://pypi.org/project/paho-mqtt/)
- [Mosquitto MQTT Broker](https://mosquitto.org/)
