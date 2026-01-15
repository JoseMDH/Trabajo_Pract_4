import paho.mqtt.client as mqtt

BROKER = "localhost"

TOPIC_PUERTA = "puerta/estado"  # 00Cerrado,01Abierto,10Cerrado,11Abierto
TOPIC_LUZ    = "luz/estado"     # 0HayLuz,1NoHayLuz

def on_connect(client, userdata, flags, rc):
    print("Conectado al broker, código:", rc)
    client.subscribe(TOPIC_PUERTA)
    client.subscribe(TOPIC_LUZ)

def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode().strip()
    print("Mensaje recibido:", topic, payload)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, 1883)
client.loop_forever()
