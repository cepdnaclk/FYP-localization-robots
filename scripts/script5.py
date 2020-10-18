import cv2 as cv
import numpy as np
import paho.mqtt.client as paho
import json
import math
import time

robots = {}
update_xy_threshold = 10       # units
update_heading_threshold = 10    # degrees

mqtt_server="68.183.188.135"
mqtt_port=1883
mqtt_keepalive=60
sub_topic_update="v1/localization/update"
sub_topic_publish="v1/localization/info"

# temp topics, for debug purposes
sub_topic_update_robot="v1/localization/update/robot"
sub_topic_create="v1/gui/create"

def update_robot(id, x, y, heading):
    #print([id, x, y, heading])
    id = int(id)

    if id in robots:
        old = robots[id];
        if( (math.sqrt(abs( pow(x-old['x'], 2) + pow(y-old['y'], 2))) >= update_xy_threshold)):

            print('Robot coordinate update received')
            #  or (abs(old['heading'] - heading) >= update_heading_threshold)
            # update the server about new coordinates
            robots[id]['x'] = x
            robots[id]['y'] = y
            robots[id]['heading'] = heading
            client.publish(sub_topic_publish, json.dumps(robots[id]), qos=1)
    else:
        # create and publish
        robots[id] =  {'id':id, 'x':x, 'y':y, 'heading':heading}
        client.publish(sub_topic_publish, json.dumps(robots[id]), qos=1)

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(sub_topic_update)
    client.subscribe(sub_topic_create)
    client.subscribe(sub_topic_update_robot)

    # Adding a robot into the data  structure - only for debug
    robots[0] = {'id':0, 'x':0.0, 'y':0.0, 'heading':0.0}


def on_message(client, userdata, msg):
    print(msg.topic+" > "+str(msg.payload, 'utf-8'))

    topic = msg.topic
    body = str(msg.payload, 'utf-8')

    if (topic==sub_topic_update):
        #print((robots))
        client.publish(sub_topic_publish, json.dumps(robots, indent=2), qos=1)

    elif (topic== sub_topic_create):    # only for testings purposes
        d=json.loads(body)
        robots[d['id']] = d
        print(robots)
    elif (topic == sub_topic_update_robot):
        d=json.loads(body)
        #print(d)
        update_robot(d['id'], d['x'], d['y'], d['heading'])


# -- MQTT Client ---------------------------------------------------------------

# Load the predefined dictionary
dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
parameters = cv.aruco.DetectorParameters_create()

# Load camera calibrations ---------------------------------------------------
cv_file = cv.FileStorage('../board/calibration_data.txt', cv.FILE_STORAGE_READ)
cameraMatrix = cv_file.getNode("K").mat()
distCoeffs = cv_file.getNode("D").mat()
cv_file.release()

print(cameraMatrix)
print(distCoeffs)

# ----------------------------------------------------------------------------

if __name__ == '__main__':

    client = paho.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(mqtt_server, mqtt_port, mqtt_keepalive)
    client.loop()
    time.sleep(2)

    print('Press "q" to quit')
    capture = cv.VideoCapture(1)

    if capture.isOpened():  # try to get the first frame
        frame_captured, frame = capture.read()
    else:
        frame_captured = False

    counter = 0;

    while frame_captured:

        if counter>=5:
            counter = 0
            client.loop()
        else:
            counter=counter+1

        # detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
        markerCorners, markerIds, rejectedCandidates = cv.aruco.detectMarkers(frame, dictionary, parameters=parameters)

        # Non-empty array of markers
        if (type(markerIds) != type(None)):

            # drawDetectedMarkers(outputImage, markerCorners, markerIds)
            cv.aruco.drawDetectedMarkers(frame, markerCorners, markerIds);

            # estimatePoseSingleMarkers(markerCorners, size_of_marker_in_real, cameraMatrix, distCoeffs, rvecs, tvecs);
            rvecs, tvecs, _objPoints = cv.aruco.estimatePoseSingleMarkers(markerCorners, 50, cameraMatrix, distCoeffs)

            for i in range(len(markerIds)):

                id = markerIds[i][0]

                coordinate = tvecs[i][0]

                x = math.floor(coordinate[0])
                y = math.floor(coordinate[1])
                rotation = math.floor((rvecs[i][0][1]/math.pi)*180.0)
                #print(id, x, y, rotation ) # rvecs[i],
                update_robot(id, x, y, 0)

                # Display marker coordinates with x,y,z axies
                cv.aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 100);

        cv.imshow('Marker Detector', frame)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        frame_captured, frame = capture.read()

    # When everything done, release the capture
    capture.release()
    cv.destroyAllWindows()
