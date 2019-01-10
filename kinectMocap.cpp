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

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL) {
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

// start kinect captor
int initSensor() {
	HRESULT hr;
	int res = 0;

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

	return res;
}

// stop kinect captor
int closeSensor() {
	// done with body frame reader
	SafeRelease(m_pBodyFrameReader);

	// close the Kinect Sensor
	if (m_pKinectSensor) {
		m_pKinectSensor->Close();
	}

	SafeRelease(m_pKinectSensor);

	return 1;
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
	int init() { return initSensor(); }
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