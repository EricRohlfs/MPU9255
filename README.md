# MPU9255
Using a cheap MPU9255 IMU for pose estimation and integration with ROS and Ruiz via arduino

This code uses the MPU9255 library: https://github.com/Bill2462/MPU9255-Arduino-Library

-----

## Content:
- MPU9255.ino: Arduino sketch for reading IMU values (Acc,Gyr,Mag) and processing the raw data into Roll, Pitch, Yaw angles. issues: Yaw angle needs debuging, need to check equations.

    <p align="center">
    <img src="https://github.com/OakLake/MPU9255/blob/master/imgs/serial_plotter_RPY.png">
    </p>

- MPU9255_rosserial_arduino.ino: Arduino sketch for communicating with ROS (on Linux) via serrial. issues: too much computation on Arduino, need to split arduino parts to only publishing raw data, and adding a processing node in ROS.

-----

## Resources

- https://github.com/Bill2462/MPU9255-Arduino-Library
- https://stanford.edu/class/ee267/misc/MPU-9255-Datasheet.pdf
- http://danceswithcode.net/engineeringnotes/rotations_in_3d/demo3D/rotations_in_3d_tool.html

