

# ------------------------------------------------------------
# Example using openCV aruco library
# ------------------------------------------------------------


import cv2 as cv
import numpy as np

# Load the predefined dictionary
dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)

# Load the dictionary that was used to generate the markers.
dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)

# Initialize the detector parameters using default values
parameters = cv.aruco.DetectorParameters_create()

if __name__ == '__main__':
    print('Press "q" to quit')
    capture = cv.VideoCapture(0)

    if capture.isOpened():  # try to get the first frame
        frame_captured, frame = capture.read()
    else:
        frame_captured = False

    while frame_captured:
        # Detect the markers in the image
        # markers = detect_markers(frame)

        # detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
        markerCorners, markerIds, rejectedCandidates = cv.aruco.detectMarkers(frame, dictionary, parameters=parameters)

        if(markerIds!= None):

            # drawDetectedMarkers(outputImage, markerCorners, markerIds)
            cv.aruco.drawDetectedMarkers(frame, markerCorners, markerIds);

            for marker in markerIds:
                print(marker)

        cv.imshow('Test Frame', frame)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        frame_captured, frame = capture.read()

    # When everything done, release the capture
    capture.release()
    cv.destroyAllWindows()



# estimatePoseSingleMarkers(markerCorners, size_of_marker_in_real, cameraMatrix, distCoeffs, rvecs, tvecs);

