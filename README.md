# Person Tracking Drone
 
## Key Functionalities and Features

* Drone detects and follows a person using Tensorflow Lite with the ssd-mobilenet-v1 model.
* Utilizes two PID controllers to smoothly determine error correction.
* Code runs on a bare Raspberry Pi 4b (no accelerator!) while achieving speeds of ~20fps (no ROS) by integrating C++ code for object detection.

## Explanation of Project

This project uses a camera on the drone to track and follow a person using OpenCV image recognition.
This code runs on the drone on a Raspberry Pi 4b companion computer.

The code ran slow with examples from online (1-3 fps), but the code was optimized to run a stable 7 fps with low latency using Python multithreading. 
Further optimization was done in January 2024 using C++ to bring the loop to around 15 - 20 fps.

Integration with Robot Operating System (ROS) in February of 2025 allowed for other packages to be used such as a GStreamer package and even AI path planning capabilities.
However, the inference time for person tracking increased back to a 7 fps result.

**Working Principle**

Direction / Yaw: 

The code returns the center of a person by taking the average of the X coordinates of the bounding boxes. Then it calculates the percentage of pixels that it is away from the center of the screen. This is passed to a PD controller and then to a drone function to turn the drone.

Forward / Backwards Movement: 

The code uses the bounding box height to determine how far a person is. It calculates a relative area using the Y distance of the bounding box. The Y axis was chosen over the X distance since the latter can be easily manipulated by spreading arms. This is passed to a P controller to determine the drone's movement.
