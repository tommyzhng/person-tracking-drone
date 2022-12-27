# Drone Project
This repository contains code and videos of my drone development progress.

The current two projects I have coded as of Dec 1st 2022:
- Person tracking / following drone
- Hoverslam Drone (Innapplicable to real life due to physical limitations like vortex ring state)

The code is tested using arducopter's SITL along with the Gazebo simulator. This reduces error and chance of failure in real life:

<p align="center">
  <img width="80%" height="80%" src="https://github.com/tommyzhng/drone/blob/master/Videos%20and%20Pictures/readme%20gifs/gazebo%20demo.gif">
</p>

## How to navigate this repository
There are three sections:
- Preplanning&Assembly
  - Some of my preplanning process like a spreadsheet for all my parts, first python project, etc
- Programming&Drone Development
  - Where all of the code for each project is stored - so far there is a hoverslam folder, a person tracking folder, and a tests folder
- Media
  - All the tests and pictures of the building/development process is stored here. The list is sorted by date.

# Explaination for each project
## Person Tracking / Following

[Access Code Location](https://github.com/tommyzhng/drone/tree/master/Programming%26Drone%20Development/personTrackingDrone)

This project uses the camera on board the drone to track and follow a person using OpenCV image recognition.
Normally, people who attempt this use an Nvidia Jetson Nano because of its fast image processing. However, I was only able to get my hands on a Raspberry Pi 4b.

The code ran super slow with examples from online (1-3 fps), but I managed to optimize my code to run at a stable 7 fps with low latency using python multithreading. 
7 calculations per second is a large improvement for the pi board to tell where I am in space relative to the drone

Example simulation using gazebo:

<p align="center">
  <img width="80%" height="80%" src="https://github.com/tommyzhng/drone/blob/master/Videos%20and%20Pictures/readme%20gifs/person%20tracking%20in%20gazebo.gif">
</p>


**How does it work?**

The code is put together from three files, main.py, tracker.py, and drone.py. main.py is where I run the code, tracker.py is where the tracking data comes from, and drone.py contains functions to move the drone.

Direction / Yaw: The code returns the center of a person by taking the average of the X coordinates of the bounding boxes. Then it calculates the percentage of pixels that it is away from the center of the screen. This is passed to a mavlink function (yaw rate) which turns the drone to face the person.

Forward / Backwards Movement: Because I do not have access to a lidar sensor, I can only use the image to determine how far a person is. The code calculates a relative area using only the Y distance of the bounding box (maxy-miny * 1). This is a better method than multiplying by the X distance because the X distance can be easily manipulated by spreading arms. The area is passed to a logic function telling the drone to stop, go forwards or backwards.

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

## Hoverslam / Suicide Burn

[Access Code Location](https://github.com/tommyzhng/drone/tree/master/Programming%26Drone%20Development/hoverslamDrone)

For this project, I wanted to see if a Space-X style hoverslam would be possible on a quad-copter by only using acceleration formulae that I learned in Grade 11 Physics class. This would greatly improve a drone's landing efficiency and speed. However, after finishing the code, I found that physical limitations such as vortex ring state was a factor that would tear my project into pieces (quite literally). Vortex ring state is a condition when a propeller is descending fast into its own downwash, resulting in an inability to produce lift. A fast descent is one of the main parts of a hoverslam, so I knew it would not work in real life. 

As a result, this project was entirely simulated within the Gazebo virtual environment, where vortex ring state did not affect the drone much (as can be seen in the video below).
<p align="center">
  <img width="80%" height="80%" src="https://github.com/tommyzhng/drone/blob/master/Videos%20and%20Pictures/readme%20gifs/hoverslam%20demo.gif">
</p>

Logic Flowchart (Click to enlarge)
<p align="center">
  <img width="80%" height="80%" src="https://github.com/tommyzhng/drone/blob/master/Videos%20and%20Pictures/readme%20gifs/hoverslam%20flowchart.png">
</p>
