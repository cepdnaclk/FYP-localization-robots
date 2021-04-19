#
# https://pypi.org/project/paho-mqtt/
#
import numpy as np
import paho.mqtt.client as paho
import json
import math
import time

mqtt_server="68.183.188.135"
mqtt_port=1883
mqtt_keepalive=60
sub_topic_update="v1/localization/update"
sub_topic_publish="v1/localization/info"

# temp topics, for debug purposes
sub_topic_update_robot="v1/localization/update/robot"
sub_topic_create="v1/gui/create"

robots = {}
update_xy_threshold = 0.5       # units
update_heading_threshold = 1    # degrees

def update_robot(id, x, y, heading):
    #print('Robot coordinate update received')
    #print([id, x, y, heading])

    if id in robots:
        old = robots[id];

        if( (math.sqrt(abs( pow(x-old['x'], 2) + pow(y-old['y'], 2))) >= update_xy_threshold) or
           (abs(old['heading'] - heading) >= update_heading_threshold) ):

            # update the server about new coordinates
            robots[id]['x'] = x
            robots[id]['y'] = y
            robots[id]['heading'] = heading
            client.publish(sub_topic_publish, json.dumps(robots[id]), qos=1)
    else:
        # create and publish
        robots[id] =  {'id':id, 'x':x, 'y':y, 'head':heading}
        client.publish(sub_topic_publish, json.dumps(robots[id]), qos=1)


def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(sub_topic_update)
    client.subscribe(sub_topic_create)
    client.subscribe(sub_topic_update_robot)

    # Adding a robot into the data  structure - only for debug
    robots[0] = {'id':0.0, 'x':0.0, 'y':0.0, 'head':0.0}

def on_message(client, userdata, msg):
    print(msg.topic+" > "+str(msg.payload, 'utf-8'))

    topic = msg.topic
    body = str(msg.payload, 'utf-8')

    if (topic==sub_topic_update):
        client.publish(sub_topic_publish, json.dumps(robots), qos=1)

    elif (topic== sub_topic_create):    # only for testings purposes
        d=json.loads(body)
        robots[d['id']] = d
        print(robots)

    elif (topic == sub_topic_update_robot):
        d=json.loads(body)
        print(d)
        update_robot(d['id'], d['x'], d['y'], d['heading'])

# -- MQTT Client ---------------------------------------------------------------
client = paho.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(mqtt_server, mqtt_port, mqtt_keepalive)

while True:
    client.loop()
    # time.sleep(1)





'''
def on_publish(mqttc, obj, mid):
    print("mid: " + str(mid))
    pass

def on_subscribe(mqttc, obj, mid, granted_qos):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))


client.on_publish = on_publish
client.on_subscribe = on_subscribe

(rc, mid) = client.publish("tuple", "bar", qos=2)

infot = client.publish("class", "bar", qos=2)
infot.wait_for_publish()


'''
