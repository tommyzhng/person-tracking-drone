# Drone Project
This repository contains code and videos of my drone development progress.

The current two projects I have coded as of Dec 1st 2022:
- Person tracking / following drone
- Hoverslam Drone (Innapplicable to real life due to physical limitations like vortex ring state)



The code is tested using arducopter's SITL along with the Gazebo simulator. This reduces error and chance of failure in real life:

<p align="center">
  <img width="70%" height="70%" src="https://github.com/tommyzhng/drone/blob/master/readme%20gifs/gazebo%20demo.gif">
</p>

## How to navigate this repository
There are three sections:
- Preplanning&Assembly
  - Some of my preplanning process like a spreadsheet for all my parts, first python project, etc
- Programming&Drone Development
  - Where all of the code for each project is stored - so far there is a hoverslam folder, a person tracking folder, and a tests folder
- Media
  - All the tests and pictures of the building/development process is stored here. The list is sorted by date.

## Explaination for each project
### Person Tracking / Following
The most generic description of this project is that it uses the camera on board the drone to track and follow a person using OpenCV image recognition.
Normally, people who attempt this use an Nvidia Jetson Nano because of its fast image processing, however, I was only able to get my hands on a Raspberry Pi 4p.

The code ran super slow with examples from online (1-3 fps), but I managed to optimize code to run at a stable 7 fps. 
7 calculations per second is enough to calculate where I am in space relative to the drone: 

<p align="center">
  <img width="70%" height="70%" src="https://github.com/tommyzhng/drone/blob/master/readme%20gifs/gazebo%20demo.gif">
</p>

### Hoverslam / Suicide Burn

