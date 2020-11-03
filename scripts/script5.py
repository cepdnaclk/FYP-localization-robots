import cv2 as cv
import numpy as np
import paho.mqtt.client as paho
import json
import math
import time
import threading

camera_id = 1

# -- Coordinate system variables -----------------------------------------------
robots = {}
update_xy_threshold = 10       # units
update_heading_threshold = 30    # degrees
x_scale = 1.0
y_scale = 1.0

# -- MQTT variables ------------------------------------------------------------
mqtt_server="68.183.188.135"
mqtt_port=1883
mqtt_keepalive=600
sub_topic_update="v1/localization/update"
sub_topic_publish="v1/localization/info"
sub_topic_color="v1/sensor/color"

# temp topics, for debug purposes
sub_topic_update_robot="v1/localization/update/robot"
sub_topic_create="v1/gui/create"

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
    x = x*x_scale
    y = y*y_scale

    if id in robots:
        old = robots[id];
        if( (math.sqrt(abs( pow(x-old['x'], 2) + pow(y-old['y'], 2))) >= update_xy_threshold)):
           #  or (abs(old['heading'] - heading) >= update_heading_threshold)):
            # update the server about new coordinates, if there is any significant difference
            robots[id]['x'] = x
            robots[id]['y'] = y
            robots[id]['heading'] = heading
            print(["update", id, x, y, heading])
            up = [ robots[id] ]
            client.publish(sub_topic_publish, json.dumps(up), qos=1)
            client.loop()
    else:
        # create and publish
        robots[id] =  {'id':id, 'x':x, 'y':y, 'heading':heading}
        color = {'id':id, 'R':0, 'G':200, 'B':0, 'ambition':50}

        client.publish(sub_topic_create, json.dumps(robots[id]), qos=1)
        client.publish(sub_topic_color, json.dumps(color), qos=1)
        client.loop()

def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT server - "+str(rc))
    client.subscribe(sub_topic_update)
    client.subscribe(sub_topic_create)
    client.subscribe(sub_topic_update_robot)
    # Adding a robot into the data  structure - only for debug
    #robots[0] = {'id':0, 'x':0.0, 'y':0.0, 'heading':0.0}

def on_message(client, userdata, msg):
    #print(msg.topic+" > "+str(msg.payload, 'utf-8'))
    topic = msg.topic
    body = str(msg.payload, 'utf-8')

    if (topic==sub_topic_update):
        # Update the coordinates of all active robots
        client.publish(sub_topic_publish, json.dumps(robots,sort_keys=True), qos=1)

    elif (topic== sub_topic_create):
        # create a robot - only for testings purposes
        d=json.loads(body)
        robots[d['id']] = d

    elif (topic == sub_topic_update_robot):
        # manually call update function - only for testing purposes
        d=json.loads(body)
        update_robot(d['id'], d['x'], d['y'], d['heading'])


# -- OpenCV Image processing ---------------------------------------------------

# Load the predefined dictionary
dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
parameters = cv.aruco.DetectorParameters_create()

# -- Load camera calibrations --------------------------------------------------

cv_file = cv.FileStorage('../board/calibration_data.txt', cv.FILE_STORAGE_READ)
cameraMatrix = cv_file.getNode("K").mat()
distCoeffs = cv_file.getNode("D").mat()
cv_file.release()

#print(cameraMatrix)
#print(distCoeffs)


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

    while frame_captured:

        markerCorners, markerIds, rejectedCandidates = cv.aruco.detectMarkers(frame, dictionary, parameters=parameters)

        # Non-empty array of markers
        if (type(markerIds) != type(None)):
            cv.aruco.drawDetectedMarkers(frame, markerCorners, markerIds);

            # estimatePoseSingleMarkers(markerCorners, size_of_marker_in_real, cameraMatrix, distCoeffs, rvecs, tvecs);
            rvecs, tvecs, _objPoints = cv.aruco.estimatePoseSingleMarkers(markerCorners, 50, cameraMatrix, distCoeffs)

            for i in range(len(markerIds)):
                id = markerIds[i][0]
                coordinate = tvecs[i][0]

                x = math.floor(coordinate[0]) - 75                      # center = 75
                y = math.floor(coordinate[1])                           # center = 0
                heading = math.floor((rvecs[i][0][1]/math.pi)*180.0)    # [-180, 180]
                #print(id, x, y, rotation ) # rvecs[i],
                update_robot(id, x, y, heading)

                # Display marker coordinates with x,y,z axies
                cv.aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 100);

        cv.imshow('Marker Detector', frame)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        frame_captured, frame = capture.read()

    # When everything done, release the capture
    capture.release()
    cv.destroyAllWindows()
