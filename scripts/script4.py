#
# https://pypi.org/project/paho-mqtt/#installation
#

import paho.mqtt.client as mqtt

mqtt_server="mqtt.eclipse.org"
mqtt_port=1883
mqtt_keepalive=60

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    client.subscribe("$SYS/#")

def on_message(client, userdata, msg):
    print(msg.topic+" > "+str(msg.payload))

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(mqtt_server, mqtt_port, mqtt_keepalive)

#client.loop_forever()

while True:
    client.loop()
    #print('*')
