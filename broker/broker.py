import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import time

# Config GPIO servo/LED
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)  # Servo pin
GPIO.setup(17, GPIO.OUT)  # LED pin
servo = GPIO.PWM(18, 50)
servo.start(0)

BROKER = "localhost"
TOPIC_SENSOR = "sensor/detectado"
TOPIC_ACTUADOR = "actuador/status"

def on_connect(client, userdata, flags, rc):
    print("Conectado al broker")
    client.subscribe(TOPIC_SENSOR)

def on_message(client, userdata, msg):
    if msg.payload.decode() == "PERSONA":
        print("Persona detectada. Abriendo puerta...")
        GPIO.output(17, GPIO.HIGH)
        servo.ChangeDutyCycle(7.5)
        time.sleep(2)
        servo.ChangeDutyCycle(2.5)  
        GPIO.output(17, GPIO.LOW) 
        client.publish(TOPIC_ACTUADOR, "Cerrando puerta...")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, 1883)
client.loop_forever()
