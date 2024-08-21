# Smart-Fan
### Smart Table Fan: Adaptive Comfort with Real-Time Environmental Sensing

## Introduction
This project introduces a Smart Table Fan, which intelligently adapts its rotation and speed based on real-time environmental data and user presence. The system leverages face detection and distance measurement to optimize airflow, enhancing user comfort while conserving energy.

## Project Philosophy🎓
The goal of this project is to fuse modern sensing technology with everyday appliances, creating smart systems that respond dynamically to environmental changes. By utilizing computer vision and data acquisition technologies, this Smart Table Fan offers an innovative approach to personalized comfort and energy efficiency.

## Tech Stack

[![Python](https://img.shields.io/badge/Python-blue?logo=python&logoColor=yellow)](https://www.python.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-green?logo=opencv&logoColor=white)](https://opencv.org/)
[![MediaPipe](https://img.shields.io/badge/MediaPipe-orange?logo=google&logoColor=white)](https://google.github.io/mediapipe/)
[![LabVIEW](https://img.shields.io/badge/LabVIEW-yellow?logo=ni-labview&logoColor=white)](https://www.ni.com/en-us/shop/labview.html)
[![Debian](https://img.shields.io/badge/Debian-red?logo=debian&logoColor=white)](https://www.debian.org/)
## Features
- **Smart Rotation**: Utilizes face detection to adjust the fan's rotation angle based on user location within a 5-meter range.
- **Distance-Based Speed Control**: Generates PWM signals based on the user's distance, enabling precise fan speed adjustments.
- **Environmental Sensing**: Monitors and adjusts fan operation based on temperature and humidity.
- **User-Friendly Interface**: Provides manual control through LabVIEW for customizable settings.
- **Energy Efficiency**: Reduces energy consumption by dynamically adjusting fan operations.

## Documentation

The block diagram above illustrates how the components of the Smart Table Fan system interact:

<img src="proposal\block diagram.png"></img>

- **Raspberry Pi**: Processes video input from the webcam for face detection using OpenCV and MediaPipe, calculates distance, and controls the fan's rotation angle.
- **Webcam**: Captures real-time video to detect user presence and measure distance.
- **LabVIEW**: Integrates data from sensors and controls the system, managing fan speed and rotation.
- **Servo Motor**: Adjusts the fan's direction based on the user's position.
- **DC Motor**: Modulates fan speed according to PWM signals from the Raspberry Pi and power supply controlled by LabVIEW.
- **NI DAQ mx**: Acquires data from various sensors and actuators, providing real-time feedback to the LabVIEW interface.
- **Rigol Variable DC Power Supply**: Modifies fan speed based on control signals from LabVIEW and the Raspberry Pi.


## Getting Started🚀
To explore the project, please follow these steps:
 1) Clone the repository.
 2) Install the necessary software and packages.
 3) Connect the hardware components as per the block diagram.
 4) Run the LabVIEW interface to start the fan and monitor its operations.

## Contributing🤝
We welcome contributions from individuals and teams interested in further developing this project. Potential areas include:
 - Improving face detection algorithms for better accuracy and speed.
 - Enhancing the PWM control for more precise fan speed adjustments.
 - Integrating additional sensors for expanded environmental monitoring.
 - Sharing insights and feedback through discussions and pull requests.

## License📄
This project is licensed under the [MIT License](https://choosealicense.com/licenses/mit/), allowing for open-source collaboration and modification. We encourage community involvement and hope this platform sparks further innovation in smart home appliances.





