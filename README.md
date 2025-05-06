# Disclaimer
This add-on is originally created by moraell, I just updated to work with the latest version of Blender

# KinectMocap4Blender
A Kinect motion capture plugin for Blender 4.4.1

## Presentation
This Blender add-on allows you to capture live movement for a human-like model in Blender, using a Microsoft Kinect V2 sensor. It is a cheap solution for homemade motion capture, and quite efficient for a free tool.

The target model must be in a standing rest pose.

It has been developped for Blender 4.4 on Windows 11.

## How to optimize the capture
It is best to have a large room with a flat floor, and a good lighting but not too bright.
The sensor must be horizontal and it is best to place it around waist height.
The tilt angle doesn't really matter as long as the device can sense the floor.
Don't get closer than 1.5 meters to the sensor, especially if you track position.
The actor's clothing shouldn't be too loose, so that each limb is clearly visible.
Use Kinect Studio to check that your workspace is optimized :
  - The floor grid (green and grey tiles) must appear and be stable
  - The actor's skeleton must not be too shaky
  - If not, try and modify the lighting or the sensor position and tilt angle.

## Demo video 

- Short demo clip : [https://youtu.be/FaTr2-vqte8]
- Demo file : [https://github.com/Kvendy-an/KinectMocap4Blender/blob/master/Demo.blend]

## Before installing the Add-on
- Install Kinect for Windows SDK 2.0

## Installing the Add-on
- Download the latest release zip
- Edit -> Preference -> Add-ons -> Add-ons Settings -> Install from Disk -> Locate the zip folder -> Press Install from Disk

## Dependencies
- Python 3.11 for Blender 4.4x
- Boost Python v1.88.0
- Kinect for Windows SDK 2.0
- Eigen 3.4.0

## Build
There are two parts in the project :
  - kinect_mocap.py : the blender add-on
  - kinectMocap4Blender.pyd : a C++ library

The library was designed to be built using Visual Studio 2022
