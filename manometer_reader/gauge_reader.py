import time
import json
import paho.mqtt.client as mqtt

with open("/data/options.json") as f:
    opt = json.load(f)

mqtt = mqtt.Client()
mqtt.connect(opt.get("mqtt_host", "core-mosquitto"),
             opt.get("mqtt_port", 1883), 60)
mqtt.loop_start()

value = 1.5  # Testwert

while True:
    mqtt.publish(opt.get("mqtt_topic", "home/manometer/pressure"), value, retain=True)
    time.sleep(opt.get("interval_sec", 10))
