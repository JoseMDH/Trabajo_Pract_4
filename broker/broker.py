import paho.mqtt.client as mqtt

BROKER = "localhost"

TOPIC_SENSOR_LUZ   = "sensor/luz"
TOPIC_SENSOR_PUERTA = "sensor/persona"
TOPIC_LED          = "actuador/led"
TOPIC_SERVO        = "actuador/servo"

# Umbral a ajustar
UMBRAL_LUZ_OSCURO = 400

def on_connect(client, userdata, flags, rc):
    print("Conectado al broker, código:", rc)
    client.subscribe(TOPIC_SENSOR_LUZ)
    client.subscribe(TOPIC_SENSOR_PUERTA)

def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode().strip()
    print("Mensaje recibido:", topic, payload)

    if topic == TOPIC_SENSOR_LUZ:
        try:
            valor = int(payload)
        except ValueError:
            print("Valor no numérico en sensor/luz:", payload)
            return

        if valor > UMBRAL_LUZ_OSCURO:
            # Está oscuro -> encender LED
            client.publish(TOPIC_LED, "on")
            print("Está oscuro, mando on al LED")
        else:
            client.publish(TOPIC_LED, "off")
            print("Hay luz, mando off al LED")

    elif topic == TOPIC_SENSOR_PUERTA:
        # Enviando Persona/NoPersona o lo que sea.
        if payload == "PERSONA":
            client.publish(TOPIC_SERVO, "o")   # open
            print("Persona detectada, mando o al SERVO")
        elif payload == "NOPERSONA":
            client.publish(TOPIC_SERVO, "c")   # close
            print("Sin persona, mando c al SERVO")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, 1883)
client.loop_forever()
