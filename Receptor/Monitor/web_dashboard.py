#!/usr/bin/env python3
"""
Dashboard Web para visualizar sensores MQTT
============================================
Página web sencilla que muestra el estado de los sensores de luz y puerta
con emojis y permite controlar manualmente la puerta.
"""

from flask import Flask, render_template
from flask_socketio import SocketIO
import paho.mqtt.client as mqtt
import threading
import json
import time

# =====================
# Configuración
# =====================
MQTT_BROKER = "localhost"
MQTT_PORT = 1883

# Topics MQTT
TOPIC_SENSOR_LUZ = "sensores/luz"
TOPIC_SENSOR_PUERTA = "sensores/puerta"
TOPIC_ACTUADOR_LUZ = "actuador/luz"
TOPIC_ACTUADOR_PUERTA = "actuador/puerta"

# Estado inicial
estado = {
    "luz": None,
    "puerta": None,
    "mqtt_connected": False
}

# =====================
# Configurar Flask y SocketIO
# =====================
app = Flask(__name__)
app.config['SECRET_KEY'] = 'secreto_sensores_2024'
# cors_allowed_origins="*" permite conexiones desde cualquier IP/dominio
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Cliente MQTT
mqtt_client = mqtt.Client()

# =====================
# Callbacks MQTT
# =====================
def on_mqtt_connect(client, userdata, flags, rc, properties=None):
    """Cuando se conecta al broker MQTT (compatible v1 y v2)"""
    if rc == 0:
        print(f"[MQTT] ✅ Conectado al broker {MQTT_BROKER}")
        client.subscribe(TOPIC_SENSOR_LUZ)
        client.subscribe(TOPIC_SENSOR_PUERTA)
        print(f"[MQTT] Suscrito a: {TOPIC_SENSOR_LUZ}, {TOPIC_SENSOR_PUERTA}")
        
        estado["mqtt_connected"] = True
        socketio.emit('mqtt_status', {'connected': True})
    else:
        print(f"[MQTT] ❌ Error de conexión, código: {rc}")
        estado["mqtt_connected"] = False
        socketio.emit('mqtt_status', {'connected': False})

def on_mqtt_disconnect(client, userdata, rc, properties=None):
    print(f"[MQTT] Desconectado, código: {rc}")
    estado["mqtt_connected"] = False
    socketio.emit('mqtt_status', {'connected': False})

def on_mqtt_message(client, userdata, msg):
    """Cuando se recibe un mensaje MQTT"""
    topic = msg.topic
    try:
        payload = msg.payload.decode('utf-8').strip()
        valor = int(payload)
    except (ValueError, UnicodeDecodeError):
        # Ignorar mensajes no numericos
        return
    
    print(f"[MQTT] 📩 Recibido: {topic} = {valor}")
    
    if topic == TOPIC_SENSOR_LUZ:
        estado["luz"] = valor
        socketio.emit('update_luz', {'valor': valor})
    elif topic == TOPIC_SENSOR_PUERTA:
        estado["puerta"] = valor
        socketio.emit('update_puerta', {'valor': valor})

# =====================
# Rutas Flask
# =====================
@app.route('/')
def index():
    """Página principal"""
    return render_template('index.html')

# =====================
# Eventos SocketIO
# =====================
@socketio.on('connect')
def handle_connect():
    """Cuando un cliente web se conecta"""
    print(f"[Web] 🌐 Cliente conectado")
    # Enviar estado actual
    socketio.emit('estado_inicial', estado)
    socketio.emit('mqtt_status', {'connected': estado["mqtt_connected"]})

@socketio.on('disconnect')
def handle_disconnect():
    """Cuando un cliente web se desconecta"""
    print("[Web] Cliente desconectado")

@socketio.on('publicar_luz')
def handle_publicar_luz(data):
    """Publicar valor en topic de luz"""
    valor = data.get('valor', 0)
    print(f"[Web] 📤 Publicando luz: {valor}")
    # Publicamos en el mismo topic de sensor para simular o controlar
    mqtt_client.publish(TOPIC_SENSOR_LUZ, str(valor))

@socketio.on('publicar_puerta')
def handle_publicar_puerta(data):
    """Publicar valor en topic de puerta"""
    valor = data.get('valor', 0)
    print(f"[Web] 📤 Publicando puerta: {valor}")
    # Publicamos en el mismo topic de sensor
    mqtt_client.publish(TOPIC_SENSOR_PUERTA, str(valor))

# =====================
# Configurar MQTT de forma robusta
# =====================
def configurar_mqtt():
    # Asignar callbacks - manejamos argumentos variables para compatibilidad paho v1/v2
    mqtt_client.on_connect = lambda c, u, f, rc, p=None: on_mqtt_connect(c, u, f, rc, p)
    mqtt_client.on_message = on_mqtt_message
    mqtt_client.on_disconnect = lambda c, u, rc, p=None: on_mqtt_disconnect(c, u, rc, p)

    # loop_start() maneja el hilo de red automáticamente y reconexiones
    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()
        print("[MQTT] Cliente iniciado en segundo plano")
    except ConnectionRefusedError:
        print(f"[MQTT] ❌ No se pudo conectar a {MQTT_BROKER}:{MQTT_PORT}. ¿Está corriendo Mosquitto?")
    except Exception as e:
        print(f"[MQTT] ❌ Error iniciando cliente: {e}")

# =====================
# Main
# =====================
if __name__ == '__main__':
    print("\n" + "="*50)
    print("🏠 Dashboard de Sensores IoT")
    print("="*50)
    
    configurar_mqtt()
    
    print(f"[Server] Iniciando servidor web...")
    print(f"[Server] Accede en: http://0.0.0.0:5000")
    print("Presiona Ctrl+C para salir\n")
    
    # Usar threading mode de socketio para compatibilidad
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
