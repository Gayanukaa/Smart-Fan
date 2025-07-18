# Smart-Fan
### Smart Fan: Adaptive Comfort with Intelligent Environmental Sensing

<img src="images\cover image.png"></img>

## Introduction

This project introduces a Smart Fan, which intelligently adapts its rotation and speed based on real-time environmental data and user presence. The system leverages computer vision for distance measurement and rotation angle to optimize airflow, enhancing user comfort while conserving energy.

[![Python](https://img.shields.io/badge/Python-blue?logo=python&logoColor=yellow)](https://www.python.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-green?logo=opencv&logoColor=white)](https://opencv.org/)
[![MediaPipe](https://img.shields.io/badge/MediaPipe-orange?logo=google&logoColor=white)](https://google.github.io/mediapipe/)
[![LabVIEW](https://img.shields.io/badge/LabVIEW-yellow?logo=ni-labview&logoColor=white)](https://www.ni.com/en-us/shop/labview.html)
[![Debian](https://img.shields.io/badge/Debian-red?logo=debian&logoColor=white)](https://www.debian.org/)

## Features
- **Distance-Based Speed Control**: Generates PWM signals based on the user's distance, enabling precise fan speed adjustments.
- **Adaptive Rotation**: The fan adjusts its rotation angle based on user location within a 4-meter range.
- **Environmental Sensing**: Monitors and adjusts fan operation based on temperature and humidity.
- **User-Friendly Interface**: Provides manual control through LabVIEW for customizable settings.
- **Energy Efficiency**: Reduces energy consumption by dynamically adjusting fan operations.

## Documentation

The block diagram below illustrates how the components of the Smart Fan system interact:

<img src="images\block diagram.png"></img>

- **Raspberry Pi**: Processes video input from the webcam for face detection using OpenCV and MediaPipe, calculates distance, and controls the fan's rotation angle.
- **Webcam**: Captures real-time video to detect user presence.
- **LabVIEW**: Integrates data from sensors and controls the system, managing fan speed.
- **Servo Motor**: Adjusts the fan's direction based on the user's position.
- **DC Motor**: Modulates fan speed according to PWM signals from the Raspberry Pi and power supply controlled by LabVIEW.
- **NI DAQ mx**: Acquires data from various sensors and actuators, providing real-time feedback to the LabVIEW interface.
- **Rigol Variable DC Power Supply**: Modifies fan speed based on control signals from LabVIEW and the Raspberry Pi.

## Distance Estimation and Servo Control

The system uses a **Mediapipe** [model](https://ai.google.dev/edge/mediapipe/solutions/vision/face_detector) for face detection, and **OpenCV** for processing the video feed from the webcamera connected to the Raspberry Pi.

- Face Detection: The algorithm detects faces in real-time, drawing bounding boxes around them.
- Distance Measurement: The area of the bounding box is used to estimate the distance to the face and angles.

## LabVIEW Control

LabVIEW processes all three sensor inputs using the NI DAQ mx to adjust the fan speed, maintains the connection with the programmable DC power supply, and integrates a dashboard to monitor real-time data and system status.

<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <td style="padding: 0;" colspan="3" align="center">
      <img src="images/interface.png" alt="Image 1" style="width: 50%; height:50%; display: block;">
     <p align="center">Dashboard displaying real-time temperature, humidity data, and indication bulbs for system status</p>
    </td>
  </tr>
  <tr>
    <td style="padding: 0.1;">
     <img src="images/main vi.png" alt="Image 2" style="width: 100%; display: block;">
     <p align="center"> Main VI of the system</p>
    </td>
    <td style="padding: 0.1;">
     <img src="images/power supply connection.png" alt="Image 3" style="width: 100%; display: block;">
     <p align="center"> Connection with the programmable DC supply</p>
    </td>
    <td style="padding: 0.1;">
     <img src="images/fan speed regulation.png" alt="Image 4" style="width: 100%; display: block;">
     <p align="center"> Varying the speed of the fan</p>
    </td>
  </tr>
</table>

## Callibration

The system requires callibration to ensure proper functioning. The following steps outline the callibration process:

<table style="width: 100%; border-collapse: collapse;">
  <tr>
    <td style="padding: 0.2;">
     <img src="images/distance callibration.png" alt="Image 2" style="width: 100%; display: block;">
     <p align="center"> Distance Callibration</p>
    </td>
    <td style="padding: 0.2;">
     <img src="images\temperature callibration.png" alt="Image 3" style="width: 100%; display: block;">
     <p align="center">Temperature Callibration</p>
    </td>
    <td style="padding: 0.2;">
     <img src="images/humidity callibration.png" alt="Image 4" style="width: 100%; display: block;">
     <p align="center">Humidity Callibration</p>
    </td>
  </tr>
</table>

## Getting Started
To explore the project, please follow these steps:
 1) Clone the repository.
 2) Install the necessary software and packages.
 3) Connect the hardware components as per the block diagram.
 4) Run the LabVIEW interface to start the fan and monitor its operations.

## License
This project is licensed under the [MIT License](https://choosealicense.com/licenses/mit/), allowing for open-source collaboration and modification.
