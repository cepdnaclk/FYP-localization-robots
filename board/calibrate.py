'''

Calibration with ArUco and ChArUco
    https://docs.opencv.org/master/da/d13/tutorial_aruco_calibration.html

OpenCV Camera Calibration
    https://medium.com/@aliyasineser/opencv-camera-calibration-e9a48bdd1844

'''

import numpy as np
import cv2
import glob


def save_coefficients(mtx, dist, path):
    # Save the camera matrix and the distortion coefficients to given path/file.
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    cv_file.write("K", mtx)
    cv_file.write("D", dist)
    cv_file.release()


def load_coefficients(path):
    # Loads camera matrix and distortion coefficients.
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    camera_matrix = cv_file.getNode("K").mat()
    dist_matrix = cv_file.getNode("D").mat()

    cv_file.release()
    return [camera_matrix, dist_matrix]


# ----------------------------------------------------------------------------

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6 * 7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

images = glob.glob('./sample/*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7, 6), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (7, 6), corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(50)  # 500

cv2.destroyAllWindows()

print('objpoints')
print(objpoints)

print('imgpoints')
print(imgpoints)

# Camera Calibration ----------------------------------------------------------

ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
# print(cameraMatrix)
# print(distCoeffs)

# Save Data
save_coefficients(cameraMatrix, distCoeffs, './calibration_data.txt')

# Load Data
cameraMatrix, distCoeffs = load_coefficients('./calibration_data.txt')

# Error estimations ----------------------------------------------------------

mean_error = 0
tot_error = 0

for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    tot_error += error

print(["total error: ", mean_error / len(objpoints)])
