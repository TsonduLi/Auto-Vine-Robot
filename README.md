# Monocular Vision-Based Robotic Autonomy in Hazardous Environments

## Overview
At the Collaborative Haptics and Robotics in Medicine Lab at Stanford, CA, our team is developing an autonomous robotic system for nuclear waste search in challenging environments. My focus is on the Sensing System, essential for the robot's perception and navigation.

## Achievements
- **Rviz Visualization System**: Implemented using Deep Learning-based semantic segmentation ([DINOv2](https://github.com/isl-org/ZoeDepth)) and depth estimation models ([MiDas](https://github.com/isl-org/MiDaS)), achieving localization error below 10% without Lidar.
- **Data Handling**: Utilized MicroPython APIs on the [OpenMV](https://openmv.io/) platform for concurrent transmission of IMU, sensor images, and laser data via MQTT, ensuring data synchronization through advanced multithreading techniques.
- **Motion Planning**: Developed a sophisticated algorithm that integrates SLAM with real-time image streaming, augmented by IMU and laser inputs for accurate object positioning and navigation.
- - **Data Management with PostgreSQL**: Enhanced the efficiency of 3D architectural mapping and robust handling of extensive datasets, enabling more effective execution of autonomous navigation tasks.

## Ongoing Work
- **Real-World Navigation**: Focusing on software and hardware integration for optimized performance in dynamic environments.

## Future Directions
Continuing to refine our algorithms for improved navigation in hazardous settings, aiming to revolutionize robotic interaction and perception in complex environments.
