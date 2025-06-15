# ECE MAE 148 - Team 2

<img src="images/jsoe_logo.png" width="300">
# Spring 2025 Final Project - Self Parking
[Final Presentation](https://docs.google.com/presentation/d/1LYtDfWw0KNp87Ibz-X-V0k7ANJAOO4ydioHknXktUfY/edit?slide=id.p#slide=id.p)
### Team Members
 - Giovanni Bernal Ramirez - MAE
 - Manasvi Boppudi - ECE
 - Yingxuan Ouyang - ECE
 - Kevin Nguyen - MAE

### Table of Contents
[- Overview](##Overview)
[- Key Features](##Key Features)
[- How to Run](##How to Run)
[- Hardware](##Hardware)
[- Challenges](##Challenges)
[- Areas of Improvement](##Areas of Improvements)
[- Acknowledgements](##Acknowledgements)
[- Course Resources](##Course Resources)
[- Other Course Deliverables](##Other Course Deliverables)
## Overview
This project was to develop a self-parallel-parking robot utilizing only a single camera and 2 2-D LiDARs. It will autonomously navigate until it detects other vehicles and starts to detect and evaluate potential parking spots.
### Main Objectives
- Object collision avoidance
- Park into parallel spaces
### Nice to Haves
- Ability to park into much tighter spaces by doing multi-point turns
- Reverse parking into perpendicular spots
## Key Features
- **Computer Vision**
- **Collision Avoidance**
## How to Run
### Requirements

## Hardware
---
### General RC Car Wiring Diagram
<img src="images/wiring_diagram.png" width="600">

### Components (May not be in Wiring Diagram)
- **Traxxas 1/10 Ford Fiesta Chassis**
- **Jetson Nano** - Main processing unit
- **FLIPSKY 75100 Pro V2.0 VESC** - Motor and steering controller
- **OAK-D Lite Camera** - Computer vision
- **DTOF LiDAR LD19** - Rear LiDAR sensor
- **SICK TiM571-2050101** - Front LiDAR sensor
### CAD
Our project relied heavily on CAD for cable management, sensor mounting & integration, and course planning.

**CAD files can be found in the `cad` folder**
#### Car CAD
<img src="images/car_cad.png" width="400">
#### Car CAD w/ FOV Overlays
<img src="images/fov_cad.png" width="400">
#### Course CAD (Car with Obstacles)
<img src="images/course_cad.png" width="400">

## Challenges
## Areas of Improvemet
## Acknowledgements
Thank you to the dedicated staff that made this course possible and helping us throughout the quarter
- **Professor - Jack Silberman**
- **TA - Alexander**
- **TA - Winston**
- **TA - Jingli**
## Course Resources
[Course Website](https://ucsd-ecemae-148.github.io/)
- Contains primary documentation and past ECE MAE 148 final projects
## Other Course Deliverables
[Donkey Car Autonomous Laps](https://youtube.com/shorts/SDO8L6csyMs)
[Donkey Car GPS Laps](https://www.youtube.com/shorts/bQW3wik1WEM?si=BMjtb4AWy1tpJWZM)
[OpenCV Lane Following](https://www.youtube.com/shorts/oZnYPm4y-rA?feature=share)


