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
#include "SimpleKalman.h"


SimpleKalman::SimpleKalman(const double dt) : dt(dt)
{
	// initialize estimate variables
	sensorNoise = .0005; // sensor noise (squared)
	u = 0.0;  // acceleration magnitude
	uNoise = 3.0; // acceleration noise

	// initialize matrices
	A << 1, 0, 0, dt, 0, 0,
		0, 1, 0, 0, dt, 0,
		0, 0, 1, 0, 0, dt,
		0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1;

	B << dt*dt / 2, dt*dt / 2, dt*dt / 2, dt*dt / 2, dt*dt / 2, dt*dt / 2;

	C << 1, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0;

	Ez << sensorNoise, 0, 0,
		0, sensorNoise, 0,
		0, 0, sensorNoise;

	Ex << dt*dt*dt*dt / 4, 0, 0, dt*dt*dt / 2, 0, 0,
		0, dt*dt*dt*dt / 4, 0, 0, dt*dt*dt / 2, 0,
		0, 0, dt*dt*dt*dt / 4, 0, 0, dt*dt*dt / 2,
		dt*dt*dt / 2, 0, 0, dt*dt, 0, 0,
		0, dt*dt*dt / 2, 0, 0, dt*dt, 0,
		0, 0, dt*dt*dt / 2, 0, 0, dt*dt;
	Ex = Ex * uNoise * uNoise;

	P = Ex;

	I.setIdentity();
}

SimpleKalman::SimpleKalman(const double dt, const double sNoise, const double u, const double uNoise) : dt(dt), u(u), uNoise(uNoise), sensorNoise(sNoise)
{
	// initialize matrices
	A << 1, 0, 0, dt, 0, 0,
		0, 1, 0, 0, dt, 0,
		0, 0, 1, 0, 0, dt,
		0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1;

	B << dt * dt / 2, dt*dt / 2, dt*dt / 2, dt*dt / 2, dt*dt / 2, dt*dt / 2;

	C << 1, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0;

	Ez << sensorNoise, 0, 0,
		0, sensorNoise, 0,
		0, 0, sensorNoise;

	Ex << dt * dt*dt*dt / 4, 0, 0, dt*dt*dt / 2, 0, 0,
		0, dt*dt*dt*dt / 4, 0, 0, dt*dt*dt / 2, 0,
		0, 0, dt*dt*dt*dt / 4, 0, 0, dt*dt*dt / 2,
		dt*dt*dt / 2, 0, 0, dt*dt, 0, 0,
		0, dt*dt*dt / 2, 0, 0, dt*dt, 0,
		0, 0, dt*dt*dt / 2, 0, 0, dt*dt;
	Ex = Ex * uNoise * uNoise;

	P = Ex;

	I.setIdentity();
}


SimpleKalman::~SimpleKalman()
{
}

void SimpleKalman::init(const double x, const double y, const double z)
{
	estimateState << x, y, z, 0, 0, 0;
}

void SimpleKalman::getFilteredState(const double x, const double y, const double z, double result[3])
{
	predict();
	update(x, y, z);
	result[0] = estimateState[0];
	result[1] = estimateState[1];
	result[2] = estimateState[2];
}

void SimpleKalman::predict()
{
	estimateState = A*estimateState + B*u;
	P = A*P*A.transpose() + Ex;
}

void SimpleKalman::update(const double x, const double y, const double z)
{
	Matrix<double, 3, 1> measure;

	measure << x, y, z;
	K = P*C.transpose()*(C*P*C.transpose() + Ez).inverse();
	estimateState += K*(measure - C*estimateState);
	P = (I - K*C)*P;
}
