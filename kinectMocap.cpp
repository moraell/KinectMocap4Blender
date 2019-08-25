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
#include "kinectMocap.h"
#include <cmath>

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL) {
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

// start kinect sensor
int initSensor(double inDt, double inSensorNoise, double inUNoise) {
	HRESULT hr;
	int res = 0;
	tilt = -100;
	dt = inDt;
	sensorNoise = inSensorNoise;
	uNoise = inUNoise;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr)) {
		return 0;
	}

	if (m_pKinectSensor) {
		// Initialize the Kinect and get coordinate mapper and the body reader
		IBodyFrameSource* pBodyFrameSource = NULL;

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr)) {
			hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
		}

		if (SUCCEEDED(hr)) {
			hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
			res = SUCCEEDED(hr);
		}

		SafeRelease(pBodyFrameSource);
	}

	if (!m_pKinectSensor || FAILED(hr)) {
		res = 0;
	}

	if (res) {
		// Initialize kalman filters
		for (int i = 0; i < 25; i++) {
			kalman[i] = NULL;
		}
	}
	return res;
}

// stop kinect sensor
int closeSensor() {
	// done with body frame reader
	SafeRelease(m_pBodyFrameReader);

	// close the Kinect Sensor
	if (m_pKinectSensor) {
		m_pKinectSensor->Close();
	}

	SafeRelease(m_pKinectSensor);

	// delete kalman data
	for (int i = 0; i < 25; i++) {
		if (kalman[i]) {
			delete kalman[i];
			kalman[i] = NULL;
		}
	}
	return 1;
}

// apply Kalman filter
int applyKalman(int jointNumber) {
	double result[3] = { 0,0,0 };

	if (kalman[jointNumber]) {
		kalman[jointNumber]->getFilteredState(joints[jointNumber].Position.X, joints[jointNumber].Position.Y, joints[jointNumber].Position.Z, result);
		joints[jointNumber].Position.X = result[0];
		joints[jointNumber].Position.Y = result[1];
		joints[jointNumber].Position.Z = result[2];
	}
	else {
		// init filter
		kalman[jointNumber] = new SimpleKalman(dt, sensorNoise, 0, uNoise);
		kalman[jointNumber]->init(joints[jointNumber].Position.X, joints[jointNumber].Position.Y, joints[jointNumber].Position.Z);
	}
	return 0;
}

// get frame (updates joints)
int updateFrame() {
	
	int res = 0;
	if (!m_pBodyFrameReader) {
		return 0;
	}

	IBodyFrame* pBodyFrame = NULL;

	HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);
	
	if (SUCCEEDED(hr)) {

		if (tilt == -100) {
			// initialize tilt angle
			Vector4 floorPlane;
			tilt = 0;
			if (SUCCEEDED(pBodyFrame->get_FloorClipPlane(&floorPlane))) {
				tilt = atan2(floorPlane.z, floorPlane.y);
			}
		}
		
		IBody* ppBodies[BODY_COUNT] = { 0 };

		hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
		
		if (SUCCEEDED(hr)) {

			for (int i = 0; i < _countof(ppBodies); i++) {

				IBody* pBody = ppBodies[i];

				if (pBody) {

					BOOLEAN bTracked = false;
					hr = pBody->get_IsTracked(&bTracked);

					if (SUCCEEDED(hr) && bTracked) {

						hr = pBody->GetJoints(_countof(joints), joints);
						
						if (SUCCEEDED(hr)) {
							for (int j = 0; j < 25; j++) {
								// compensate tilt
								double height = joints[j].Position.Y * cos(tilt) + joints[j].Position.Z * sin(tilt);
								double depth = joints[j].Position.Z * cos(tilt) - joints[j].Position.Y * sin(tilt);
								joints[j].Position.Y = height;
								joints[j].Position.Z = depth;

								// apply kalman filter to each joint
								applyKalman(j);
							}
							res = 1;
						}
					}
				}
			}
		}

	}

	SafeRelease(pBodyFrame);

	return res;
}

struct Sensor {
	tuple getJoint(int jointNumber) { return make_tuple(joints[jointNumber].Position.X, joints[jointNumber].Position.Y, joints[jointNumber].Position.Z, static_cast<int>(joints[jointNumber].TrackingState)); }
	int init(double dt, double sNoise, double uNois) { return initSensor(dt, sNoise, uNois); }
	int close() { return closeSensor(); }
	int update() { return updateFrame(); }
};

BOOST_PYTHON_MODULE(kinectMocap4Blender) {
	class_<Sensor>("Sensor")
		.def("init", &Sensor::init)
		.def("close", &Sensor::close)
		.def("update", &Sensor::update)
		.def("getJoint", &Sensor::getJoint)
	;
}