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

# =====================
# Configuración
# =====================
MQTT_BROKER = "localhost"
MQTT_PORT = 1883

# Topics MQTT
TOPIC_SENSOR_LUZ = "sensores/luz"
TOPIC_SENSOR_PUERTA = "sensores/puerta"

# Estado inicial
estado = {
    "luz": None,
    "puerta": None
}

# =====================
# Configurar Flask y SocketIO
# =====================
app = Flask(__name__)
app.config['SECRET_KEY'] = 'secreto_sensores_2024'
socketio = SocketIO(app, cors_allowed_origins="*")

# Cliente MQTT
mqtt_client = mqtt.Client()

# =====================
# Callbacks MQTT
# =====================
def on_mqtt_connect(client, userdata, flags, rc):
    """Cuando se conecta al broker MQTT"""
    if rc == 0:
        print(f"[MQTT] Conectado al broker {MQTT_BROKER}")
        client.subscribe(TOPIC_SENSOR_LUZ)
        client.subscribe(TOPIC_SENSOR_PUERTA)
        print(f"[MQTT] Suscrito a: {TOPIC_SENSOR_LUZ}, {TOPIC_SENSOR_PUERTA}")
    else:
        print(f"[MQTT] Error de conexión, código: {rc}")

def on_mqtt_message(client, userdata, msg):
    """Cuando se recibe un mensaje MQTT"""
    topic = msg.topic
    try:
        payload = msg.payload.decode('utf-8').strip()
        valor = int(payload)
    except (ValueError, UnicodeDecodeError):
        print(f"[MQTT] Error parseando mensaje: {msg.payload}")
        return
    
    print(f"[MQTT] Recibido: {topic} = {valor}")
    
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
    print("[Web] Cliente conectado")
    # Enviar estado actual
    socketio.emit('estado_inicial', estado)

@socketio.on('disconnect')
def handle_disconnect():
    """Cuando un cliente web se desconecta"""
    print("[Web] Cliente desconectado")

@socketio.on('publicar_luz')
def handle_publicar_luz(data):
    """Publicar valor en topic de luz"""
    valor = data.get('valor', 0)
    print(f"[Web] Publicando luz: {valor}")
    mqtt_client.publish(TOPIC_SENSOR_LUZ, str(valor))

@socketio.on('publicar_puerta')
def handle_publicar_puerta(data):
    """Publicar valor en topic de puerta"""
    valor = data.get('valor', 0)
    print(f"[Web] Publicando puerta: {valor}")
    mqtt_client.publish(TOPIC_SENSOR_PUERTA, str(valor))

# =====================
# Iniciar MQTT en hilo separado
# =====================
def iniciar_mqtt():
    """Inicia la conexión MQTT"""
    mqtt_client.on_connect = on_mqtt_connect
    mqtt_client.on_message = on_mqtt_message
    
    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_forever()
    except Exception as e:
        print(f"[MQTT] Error: {e}")

# =====================
# Main
# =====================
if __name__ == '__main__':
    print("\n" + "="*50)
    print("🏠 Dashboard de Sensores IoT")
    print("="*50)
    
    # Iniciar MQTT en hilo separado
    mqtt_thread = threading.Thread(target=iniciar_mqtt, daemon=True)
    mqtt_thread.start()
    
    print(f"[Server] Iniciando en http://localhost:5000")
    print("Presiona Ctrl+C para salir\n")
    
    # Iniciar servidor web
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
