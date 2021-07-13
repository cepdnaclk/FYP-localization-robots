import cv2 as cv
import numpy as np
import paho.mqtt.client as paho
import json
import math
import time
import yaml
from time import sleep
# import threading

CONFIG_MQTT = 'config-mqtt.yaml'
CONFIG_MAPPING = 'config-mapping.yaml'
CONFIG_CAMERA = 'board/calibration_data.txt'

camera_id = 0

capture_interval = 0.005
capture_skips = 10

fps = 1/(capture_interval*capture_skips)

print('fps: ', fps)

# -- Coordinate system variables -----------------------------------------------
robots = {}
update_xy_threshold = 10  # units
update_heading_threshold = 2  # degrees

with open(CONFIG_MAPPING, 'r') as file:
    mapping_data = yaml.load(file)
    # print(mapping_data)
    REFERENCE_POINTS = mapping_data['reference_points']
    DEST_POINTS = mapping_data['dest']

# -- MQTT variables -----------------------------------------------------------

sub_topic_update = "v1/localization/update/?"
sub_topic_publish = "v1/localization/update"
sub_topic_create = "v1/robot/create"
# temp topics, for debug purposes
# sub_topic_update_robot="v1/localization/update/robot"

with open(CONFIG_MQTT, 'r') as file:
    mqtt_data = yaml.load(file)
    print(mqtt_data)
    mqtt_server = mqtt_data['mqtt_server']
    mqtt_port = mqtt_data['mqtt_port']
    mqtt_keepalive = mqtt_data['mqtt_keepalive']

def transXY(camX, camY):
    # [x,y] for TopLeft, BottomLeft, BottomRight, TopRight
    # REFERENCE_POINTS=[[-620,-595],[-630,500],[640,540],[630,-600]]
    # DEST_POINTS=[[-90,90],[-90,-90],[90,-90],[90,90]]

    REFERENCE=np.float32(REFERENCE_POINTS)
    DEST=np.float32(DEST_POINTS)
    transMatrix=cv.getPerspectiveTransform(REFERENCE,DEST)
    projected=np.dot(transMatrix, np.array([camX,camY,1]))
    # print(projected)
    return projected


# -- MQTT loop thread function - NOT WORKING FOR NOW ---------------------------
def mqtt_loop(client):
    client.loop()


def mqtt_setup():
    client = paho.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(mqtt_server, mqtt_port, mqtt_keepalive)
    time.sleep(2)
    client.loop()
    return client


def update_robot(id, x, y, heading):
    id = int(id)
    x = round(x, 2)
    y = round(y, 2)

    if id in robots:
        old = robots[id]
        update_queue = []
        if ((math.sqrt(abs(pow(x - old['x'], 2) + pow(y - old['y'], 2))) >= update_xy_threshold)) or (abs(old['heading'] - heading) >= update_heading_threshold):

            # update the server about new coordinates, if there is any significant difference
            robots[id]['id'] = id
            robots[id]['x'] = x
            robots[id]['y'] = y
            robots[id]['heading'] = heading
            robots[id]['reality'] = 'R'
            update_queue.append(robots[id])
            client.loop()

        if (len(update_queue) > 0):
            client.publish(sub_topic_publish, json.dumps(update_queue), qos=1)
            print(['Loc:', update_queue])

    else:
        # create and publish
        robots[id] = {'heading': heading, 'id': id, 'x': x, 'y': y, 'reality': 'R'}

        client.publish(sub_topic_create, json.dumps(robots[id]), qos=1)
        print(['Create', robots[id]])

        # color = {'id':id, 'R':0, 'G':200, 'B':0, 'ambition':50}
        # client.publish(sub_topic_color, json.dumps(color), qos=1)
        client.loop()


def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT server - " + str(rc))
    client.subscribe(sub_topic_update)
    # client.subscribe(sub_topic_update_robot)

    # Adding a robot into the data  structure - only for debug
    # robots[0] = {'id':0, 'x':0.0, 'y':0.0, 'heading':0.0}


def on_message(client, userdata, msg):
    # print(msg.topic+" > "+str(msg.payload, 'utf-8'))
    topic = msg.topic
    body = str(msg.payload, 'utf-8')

    if (topic == sub_topic_update):
        # Update the coordinates of all active robots
        client.publish(sub_topic_publish, json.dumps(robots, sort_keys=True), qos=1)

    # elif (topic == sub_topic_update_robot):
    #     # manually call update function - only for testing purposes
    #     d=json.loads(body)
    #     update_robot(d['id'], d['x'], d['y'], d['heading'])


# -- OpenCV Image processing ---------------------------------------------------

# Load the predefined dictionary
dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
parameters = cv.aruco.DetectorParameters_create()

# -- Load camera calibrations --------------------------------------------------

cv_file = cv.FileStorage(CONFIG_CAMERA, cv.FILE_STORAGE_READ)
cameraMatrix = cv_file.getNode("K").mat()
distCoeffs = cv_file.getNode("D").mat()
cv_file.release()

# print(cameraMatrix)
# print(distCoeffs)


# ******************************************************************************
# ******************************************************************************

if __name__ == '__main__':

    # Starts MQTT client
    client = mqtt_setup()

    '''
    try:
        t1 = threading.Thread(target=mqtt_loop, args=(client,))
        t1.start()
    except:
       print("Error: unable to start thread")
    '''

    print('Press "q" to quit')

    capture = cv.VideoCapture(0)
    if capture.isOpened():  # try to get the first frame
        frame_captured, frame = capture.read()
    else:
        frame_captured = False

    i = 0

    while frame_captured:
        frame_captured, frame = capture.read()

        # print(i)
        i += 1

        if i < capture_skips:
            # skip 50% of frames
            sleep(capture_interval)
            continue

        elif i==capture_skips:
            # Reset the counter and process the frame
            i = 0

        markerCorners, markerIds, rejectedCandidates = cv.aruco.detectMarkers(frame, dictionary, parameters=parameters)

        # Non-empty array of markers
        if (type(markerIds) != type(None)):
            cv.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)

            # estimatePoseSingleMarkers(markerCorners, size_of_marker_in_real, cameraMatrix, distCoeffs, rvecs, tvecs)
            rvecs, tvecs, _objPoints = cv.aruco.estimatePoseSingleMarkers(markerCorners, 50, cameraMatrix, distCoeffs)

            for i in range(len(markerIds)):
                id = markerIds[i][0]
                coordinate = tvecs[i][0]

                x = math.floor(coordinate[0])  # center = 0
                y = math.floor(coordinate[1])  # center = 0
                heading = math.floor(-1 * ((rvecs[i][0][1] / math.pi) * 180.0) + 90)  # [-180, 180]

                res = transXY(x,y)
                print(id, x, y, ">", res[0], res[1] ) # rvecs[i],

                update_robot(id, res[0], res[1], heading)

                # Display marker coordinates with x,y,z axies
                cv.aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 100)

        cv.imshow('Marker Detector', frame)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    capture.release()
    cv.destroyAllWindows()
