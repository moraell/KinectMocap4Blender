# KinectMocap4Blender
A Kinect motion capture plugin for Blender

## Presentation
This Blender add-on allows you to capture live movement for a human-like model in Blender, using a Microsoft Kinect V2 sensor (not compatible with Kinect 360). It is a cheap solution for homemade motion capture, and quite efficient for a free tool.

The target model must be in a standing rest pose.

It has been developped for Blender 2.79 and 2.8x (2.80 and 2.81) on Windows 10.

### A few words about the project genesis
A few monthes ago, I browsed the web in order to find a way to setup my own homemade capture studio with a Kinect sensor, for a videogame I'm working on. I am not a 3D artist and a real noob with Blender, so I wanted something to help me with the animation process. I found a few solutions but they were either very expensive (and not working very well anyway, at least the demos I tried) or not fitting my needs (not working on Windows 10 or requiring a specific armature that just wouldn't do with my target model). So I decided to learn Python, dig into Blender documentation (and I think I broke a few shovels in the process ;)), and here is the result.

## How to optimize the capture
It is best to have a large room with a flat floor, and a good lighting but not toot bright.
The sensor must be horizontal and it is best to place it around waist height.
The tilt angle doesn't really matter as long as the device can sense the floor.
Don't get closer than 1.5 meters to the sensor, especially if you track position.
The actor's clothing shouldn't be too loose, so that each limb is clearly visible.
Use Kinect Studio to check that your workspace is optimized :
  - The floor grid (green and grey tiles) must appear and be stable
  - The actor's skeleton must not be too shaky
  - If not, try and modify the lighting or the sensor position and tilt angle.

## Demo and configuration videos
Presentation : [https://youtu.be/Zt8gJzSNSbw]

Short demo clip : [https://youtu.be/cdGMrhrUsIs]

Version 1.4 presentation : [https://youtu.be/sFht6XcLZSo]


## Install from release archive
- Install Kinect for Windows SDK 2.0 if you haven't already
- Download the latest release zip archive from github ([https://github.com/moraell/KinectMocap4Blender/releases]) and unpack the files (kinecp_mocap.py and kinectMocap4Blender.pyd) corresponding to your version of Blender in Blender addons directory.
Consult Blender documentation for more information on plugin installation [https://docs.blender.org/].

## Dependencies
- Python 3.5.3 (for Blender 2.79 builds), 3.7 for Blender 2.8x
- Boost Python v1.67.0 or v1.69.0 [https://www.boost.org/]
- Kinect for Windows SDK 2.0 [https://www.microsoft.com/en-us/download/details.aspx?id=44561]

## Build
There are two parts in the project :
  - kinect_mocap.py : the blender add-on (different for Blender 2.79 and 2.8x)
  - kinectMocap4Blender.pyd : a C++ library (linking is different for 2.79 and 2.8x for different python and Boost versions)

The library was designed to be built using Visual Studio 2017 (only the Release configuration has been properly configured at the moment).

## Current progress
The project is currently in version 1.4. 
