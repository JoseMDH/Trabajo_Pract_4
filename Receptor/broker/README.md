# MQTT-LoRa Bridge

Sistema de gateway bidireccional entre MQTT y LoRa usando:
- **Raspberry Pi**: Ejecuta el broker MQTT y el bridge
- **Arduino MKR WAN 1310**: Transceptor LoRa conectado por USB/Serial

## Arquitectura

```
┌─────────────────────────────────────────────────────────────────────┐
│                          Raspberry Pi                                │
│  ┌─────────────┐         ┌──────────────────────────────────────┐   │
│  │   Mosquitto │◄───────►│        mqtt_lora_bridge.py           │   │
│  │   (Broker)  │  MQTT   │                                      │   │
│  └─────────────┘         └──────────────┬───────────────────────┘   │
│                                         │ Serial (USB)              │
└─────────────────────────────────────────┼───────────────────────────┘
                                          │
                         ┌────────────────▼────────────────┐
                         │     Arduino MKR WAN 1310        │
                         │        (Gateway.ino)            │
                         │                                 │
                         │  LoRa 868MHz                    │
                         └────────────────┬────────────────┘
                                          │ LoRa RF
                    ┌─────────────────────┼─────────────────────┐
                    │                     │                     │
           ┌────────▼────────┐   ┌────────▼────────┐   ┌────────▼────────┐
           │   Nodo Sensor   │   │   Nodo Sensor   │   │    Actuador     │
           │    (0x06)       │   │    (0x07)       │   │    (0x08)       │
           └─────────────────┘   └─────────────────┘   └─────────────────┘
```

## Protocolo Serial

Los mensajes entre la Raspberry y el Arduino usan el siguiente formato:

```
┌─────┬──────┬───────────┬───────┬─────────────┬─────────┬─────┐
│ STX │ Tipo │ TopicLen  │ Topic │ PayloadLen  │ Payload │ ETX │
│0x02 │ 1B   │    1B     │  nB   │     1B      │   nB    │0x03 │
└─────┴──────┴───────────┴───────┴─────────────┴─────────┴─────┘
```

### Tipos de mensaje:
- `R` (0x52): Mensaje recibido de LoRa (Arduino → Raspberry)
- `T` (0x54): Mensaje a transmitir por LoRa (Raspberry → Arduino)
- `A` (0x41): ACK
- `N` (0x4E): NACK
- `S` (0x53): Estado del sistema

### Payload de mensajes LoRa recibidos (tipo 'R'):
```
┌────────┬──────┬─────┬──────────────────┐
│ Sender │ RSSI │ SNR │   Datos reales   │
│   1B   │  1B  │ 1B  │       nB         │
└────────┴──────┴─────┴──────────────────┘
```

## Instalación

### En la Raspberry Pi:

1. **Instalar Mosquitto (broker MQTT)**:
   ```bash
   sudo apt update
   sudo apt install mosquitto mosquitto-clients
   sudo systemctl enable mosquitto
   sudo systemctl start mosquitto
   ```

2. **Instalar dependencias Python**:
   ```bash
   pip3 install -r requirements.txt
   ```

3. **Configurar el puerto serie** (editar mqtt_lora_bridge.py):
   ```python
   SERIAL_PORT = "/dev/ttyACM0"  # Ajustar según tu sistema
   ```

4. **Ejecutar el bridge**:
   ```bash
   python3 mqtt_lora_bridge.py
   ```

### En el Arduino MKR WAN 1310:

1. Abrir `Gateway/Gateway.ino` en Arduino IDE
2. Configurar la dirección del dispositivo si es necesario
3. Subir el sketch al Arduino

## Uso

### Topics MQTT disponibles:

| Topic | Dirección | Descripción |
|-------|-----------|-------------|
| `lora/rx` | Arduino → MQTT | Mensajes raw recibidos por LoRa (JSON) |
| `lora/tx` | MQTT → Arduino | Enviar mensajes por LoRa |
| `sensores/luz` | Arduino → MQTT | Datos del sensor de luz |
| `sensores/puerta` | Arduino → MQTT | Estado de la puerta |
| `actuador/comando` | MQTT → Arduino | Comandos para actuadores |

### Ejemplos:

**Enviar mensaje a nodo 0x06**:
```bash
mosquitto_pub -t "lora/tx" -m "Hola desde MQTT"
```

**Enviar a dirección específica** (prefijo @XX/):
```bash
# Publicar en topic "actuador/comando" dirigido a 0x08
mosquitto_pub -t "@08/actuador/comando" -m "ON"
```

**Suscribirse a todos los mensajes LoRa**:
```bash
mosquitto_sub -t "lora/rx" -v
```

**Suscribirse a sensor de luz**:
```bash
mosquitto_sub -t "sensores/luz" -v
```

## Formato de mensajes LoRa

El payload LoRa contiene:
```
┌─────────────────┬──────┬─────────────────┐
│  Topic (string) │ NULL │  Payload (raw)  │
└─────────────────┴──────┴─────────────────┘
```

Esto permite que los nodos remotos especifiquen en qué topic MQTT se publicarán sus datos.

## Troubleshooting

- **Puerto serie no encontrado**: Verificar con `ls /dev/tty*` y ajustar `SERIAL_PORT`
- **Permisos del puerto**: `sudo usermod -a -G dialout $USER` y reiniciar sesión
- **Broker MQTT no conecta**: Verificar que Mosquitto esté corriendo: `systemctl status mosquitto`
