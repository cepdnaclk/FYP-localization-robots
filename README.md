[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# FYP-localization-robots

Python repository for address the Localization problem of Swarm Robotics

### Requirements

Please install following pip packages if they aren't pre-installed

```
pip install numpy

pip install aruco
pip install paho-mqtt
pip3 install opencv-python
pip3 install opencv-contrib-python
```

or use following command

```
pip install -r requirement.txt
```

You need to copy and rename the following files in _scripts_ directory and fill the necessary configuration details before run the scripts.

- config-mapping_sample.yaml INTO config-mapping.yaml
- config-mqtt_sample.yaml INTO config-mqtt.yaml

### Run the scripts

You can try scripts on the ./scripts/ directory

### Build a executable file (not working so far)

Install PyInstaller from PyPI:

```
pip install pyinstaller
```

Go to your program’s directory and run:

```
pyinstaller --onefile script.py
```

After, please make sure to copy the './board' folder into the directory which executes the exe file.

### Read More

- [ar-markers 0.5.0](https://pypi.org/project/ar-markers/)
- [Augmented Reality using ArUco Markers in OpenCV (C++ / Python)](https://www.learnopencv.com/augmented-reality-using-aruco-markers-in-opencv-c-python/)
- [OpenCV: Detection of ArUco Markers](https://docs.opencv.org/trunk/d5/dae/tutorial_aruco_detection.html)
- [OpenCV: Detection of ArUco Boards](https://docs.opencv.org/master/db/da9/tutorial_aruco_board_detection.html)
- [Calibrating the board](https://mecaruco2.readthedocs.io/en/latest/notebooks_rst/Aruco/sandbox/ludovic/aruco_calibration_rotation.html)
