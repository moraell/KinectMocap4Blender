# KinectMocap4Blender
A Kinect motion capture plugin for Blender

## Presentation
This Blender add-on allows you to capture live movement for a human-like model in Blender, using a Microsoft Kinect V2 sensor. Although not precise, the result makes a good working base to reproduce real life movement after refining it manually. It is a cheap solution for homemade motion capture.

The target model must be in a standing rest pose.

It has been developped for Blender 2.79 Windows 10. There is also a version for Blender 2.80 but I advise strongly against using it (it has to stay in an experimental state until Blender 2.80 is officially out of Beta).

### A few words about the project genesis
A few monthes ago, I browsed the web in order to find a way to setup my own homemade capture studio with a Kinect sensor, for a videogame I'm working on. I am not a 3D artist and a real noob with Blender, so I wanted something to help me with the animation process. I found a few solutions but they were either very expensive (and not working very well anyway, at least the demos I tried) or not fitting my needs (not working on Windows 10 or requiring a specific armature that just wouldn't do with my target model). So I decided to learn Python, dig into Blender documentation (and I think I broke a few shovels in the process ;)), and here is the result.

## Demo and configuration video
[https://youtu.be/Zt8gJzSNSbw]

## Install from release archive
- Install Kinect for Windows SDK 2.0 if you haven't already
- Download the release archive and unpack the files (kinecp_mocap.py and kinectMocap4Blender.pyd) corresponding to your version of Blender in Blender addons directory.
Consult Blender documentation for more information on plugin installation. [https://docs.blender.org/]

## Dependencies
- Python 3.5.3 (for Blender 2.79 builds), 3.7 for Blender 2.80
- Boost Python v1.69.0 [https://www.boost.org/]
- Kinect for Windows SDK 2.0 [https://www.microsoft.com/en-us/download/details.aspx?id=44561]

## Build
There are two parts in the project :
  - kinect_mocap.py : the blender add-on (different for Blender 2.79 and 2.80)
  - kinectMocap4Blender.pyd : a C++ library
The library was designed to be built using Visual Studio 2017 (only the Release configuration has been properly configured at the moment).

## Current progress
The project is currently in version 1.1. 
