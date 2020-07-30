import cv2 as cv
import numpy as np

# Load the predefined dictionary
dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)

# Load the dictionary that was used to generate the markers.
dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)

# Initialize the detector parameters using default values
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
    print('Press "q" to quit')
    capture = cv.VideoCapture(0)

    if capture.isOpened():  # try to get the first frame
        frame_captured, frame = capture.read()
    else:
        frame_captured = False

    while frame_captured:

        # detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
        markerCorners, markerIds, rejectedCandidates = cv.aruco.detectMarkers(frame, dictionary, parameters=parameters)

        #print(type(markerIds))

        # Non-empty array of markers
        if (type(markerIds) != type(None)):

            # drawDetectedMarkers(outputImage, markerCorners, markerIds)
            cv.aruco.drawDetectedMarkers(frame, markerCorners, markerIds);

            # estimatePoseSingleMarkers(markerCorners, size_of_marker_in_real, cameraMatrix, distCoeffs, rvecs, tvecs);
            rvecs, tvecs, _objPoints = cv.aruco.estimatePoseSingleMarkers(markerCorners, 50, cameraMatrix, distCoeffs)

            #for i in range(len(markerIds)):
                #print(markerIds[i], rvecs[i], tvecs[i])
                
                # Display marker coordinates with x,y,z axies
                #cv.aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 100);

        cv.imshow('Marker Detector', frame)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        frame_captured, frame = capture.read()

    # When everything done, release the capture
    capture.release()
    cv.destroyAllWindows()
