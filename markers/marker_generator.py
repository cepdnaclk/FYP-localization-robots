import cv2 as cv
import numpy as np

# Load the predefined dictionary
dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)

# Generate the marker

markers = [int(x) for x in range(0, 10)]
image_size = 600

for m in markers:
    markerImage = np.zeros((image_size, image_size), dtype=np.uint8)

    # drawMarkers(dict, marker_id, pixel_size, output_image, border_width)
    markerImage = cv.aruco.drawMarker(dictionary, m, image_size, markerImage, 1);
    cv.imwrite("./generated/marker" + str(m) + ".png", markerImage);
