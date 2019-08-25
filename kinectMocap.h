/*
Copyright 2019 Morgane Dufresne

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
he Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#pragma once

#include <Kinect.h>
#include "SimpleKalman.h"

#define BOOST_PYTHON_STATIC_LIB
#include <boost/python.hpp>

using namespace boost::python;

// Current Kinect
IKinectSensor*          m_pKinectSensor;
double					tilt;

Joint					joints[JointType_Count];

// Body reader
IBodyFrameReader*       m_pBodyFrameReader;

// Kalman filters data
SimpleKalman*			kalman[25];
double					sensorNoise, uNoise;

// store framerate
double					dt;