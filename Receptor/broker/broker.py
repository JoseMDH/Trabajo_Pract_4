import paho.mqtt.client as mqtt

BROKER = "localhost"

TOPIC_1 = "puerta/estado"     # 00Cerrado,01Abierto,10Cerrado,11Abierto
TOPIC_0    = "luz/estado"     # 0HayLuz,1NoHayLuz

def on_connect(client, userdata, flags, rc):
    print("Conectado al broker, código:", rc)
    client.subscribe(TOPIC_1)
    client.subscribe(TOPIC_0)

def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode().strip()
    print("Mensaje recibido:", topic, payload)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, 1883)
client.loop_forever()
