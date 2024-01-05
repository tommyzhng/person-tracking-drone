# Person Tracking Drone

## How to navigate this repository

- TensorFlow Method:
  - Tensorflow / Tensorflow lite methods of person detection
- Videos and Pictures:
  - Pictures and videos of the development.
 
## Key Functionalities and Features

* Drone detects and follows a person using Tensorflow Lite with the ssd-mobilenet-v1 model.
* Utilizes two PID controllers to smoothly determine error correction.
* Code runs on a bare Raspberry Pi 4b (no accelerator!) while achieving speeds of ~20fps by integrating C++ code for object detection.

## Explanation of Project

[Access Code Location](https://github.com/tommyzhng/person-tracking-drone/tree/master/tensorflowMethod)

This project uses a camera on the drone to track and follow a person using OpenCV image recognition.
This code runs on the drone on a Raspberry Pi 4b companion computer.

The code ran slow with examples from online (1-3 fps), but the code was optimized to run a stable 7 fps with low latency using Python multithreading. 
Further optimization was done in January 2024 using C++ to bring the loop to around 15 - 20 fps.

**Working Principle**

The code is assembled from main.py, drone.py, tracker.py, keyboard.py and a C++ shared file named person_tracking*.so.

* main.py - where code is run, initializes every needed library.
* drone.py - definitions of drone functions, MAVlink commands, etc.
* tracker.py - obtaining video feed and person tracking (using Python).
* keyboard.py - various keystrokes that call different functions.

* C++ person_tracking.so - optimized version of person detection that runs at a peak of 20fps on a bare Raspberry Pi (no USB accelerator)

The code was tested with the Gazebo simulator:

<p align="center">
  <img width="80%" height="80%" src="https://github.com/tommyzhng/drone/blob/master/Videos%20and%20Pictures/readme%20gifs/person%20tracking%20in%20gazebo.gif">
</p>

Direction / Yaw: 

The code returns the center of a person by taking the average of the X coordinates of the bounding boxes. Then it calculates the percentage of pixels that it is away from the center of the screen. This is passed to a PD controller and then to a drone function to turn the drone.

Forward / Backwards Movement: 

The code uses the bounding box height to determine how far a person is. It calculates a relative area using the Y distance of the bounding box. The Y axis was chosen over the X distance since the latter can be easily manipulated by spreading arms. This is passed to a P controller to determine the drone's movement.

**Person Tracking Demo:** 

[Full Video](https://github.com/tommyzhng/drone/blob/master/Videos%20and%20Pictures/7%20yaw%20tracking%20person.mp4)
(embed .gif takes some time to load)
<p align="center">
  <img src="https://github.com/tommyzhng/drone/blob/master/Videos%20and%20Pictures/readme%20gifs/person%20tracking%20demo.gif">
</p>

Logic Flowchart (outdated)
<p align="center">
  <img src="https://github.com/tommyzhng/drone/blob/master/Videos%20and%20Pictures/readme%20gifs/person%20tracking%20flowchart.png">
</p>
