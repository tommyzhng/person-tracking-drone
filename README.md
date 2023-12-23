# Person Tracking Drone

## How to navigate this repository
There are three sections:
- Preplanning&Assembly
  - Some of my preplanning process like a spreadsheet for all my parts, first python project, etc
- Programming&Drone Development
  - Where all of the code for each project is stored - so far there is a hoverslam folder, a person tracking folder, and a tests folder
- Media
  - All the tests and pictures of the building/development process is stored here. The list is sorted by date.


## Explaination of Project

[Access Code Location](https://github.com/tommyzhng/Drone-Projects/tree/master/Projects/personTrackingDrone)

This project uses the camera on board the drone to track and follow a person using OpenCV image recognition.
Normally, people who attempt this use an Nvidia Jetson Nano because of its fast image processing. However, I was only able to get my hands on a Raspberry Pi 4b.

The code ran super slow with examples from online (1-3 fps), but I managed to optimize my code to run at a stable 7 fps with low latency using python multithreading. 
7 calculations per second is a large improvement for the pi board to tell where I am in space relative to the drone.

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
