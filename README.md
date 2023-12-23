# Person Tracking Drone

## How to navigate this repository

- Mediapipe & Tensorflow Method:
  - Different methods that can be applied for person tracking and bounding box generation
- Videos and Pictures:
  - Pictures and videos of the development


## Explanation of Project

[Access Code Location](https://github.com/tommyzhng/Drone-Projects/tree/master/Projects/personTrackingDrone)

This project uses the camera on board the drone to track and follow a person using OpenCV image recognition.
This code runs on the drone on a raspberry pi 4b companion computer.

The code ran slow with examples from online (1-3 fps), but the code was optimized to run a stable 7 fps with low latency using Python multithreading. 

Example simulation using gazebo:

<p align="center">
  <img width="80%" height="80%" src="https://github.com/tommyzhng/drone/blob/master/Videos%20and%20Pictures/readme%20gifs/person%20tracking%20in%20gazebo.gif">
</p>


**How it works**

The code is put together from three files, main.py, tracker.py, and drone.py. main.py is where I run the code, tracker.py is where the tracking data comes from, and drone.py contains functions to move the drone.

Direction / Yaw: The code returns the center of a person by taking the average of the X coordinates of the bounding boxes. Then it calculates the percentage of pixels that it is away from the center of the screen. This is passed to a mavlink function (yaw rate) which turns the drone to face the person.

Forward / Backwards Movement: The code uses the bounding box height to determine how far a person is. It calculates a relative area using only the Y distance of the bounding box (maxy-miny * 1). The Y axis was chosen over the X distance since the latter can be easily manipulated by spreading arms. This is passed to logic (which can be improved) to determine the movement of the drone.

**Person Tracking Demo:** 

[Full Video](https://github.com/tommyzhng/drone/blob/master/Videos%20and%20Pictures/7%20yaw%20tracking%20person.mp4)
(embed .gif takes some time to load)
<p align="center">
  <img src="https://github.com/tommyzhng/drone/blob/master/Videos%20and%20Pictures/readme%20gifs/person%20tracking%20demo.gif">
</p>

Logic Flowchart (Click to enlarge)
<p align="center">
  <img src="https://github.com/tommyzhng/drone/blob/master/Videos%20and%20Pictures/readme%20gifs/person%20tracking%20flowchart.png">
</p>
