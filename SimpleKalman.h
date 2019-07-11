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
/* A Kalman filter implementation tailored to Kinect V2 joint tracking */
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::Matrix;

#pragma once

class SimpleKalman
{
public:
	SimpleKalman(const double dt);
	SimpleKalman(const double dt, const double sNoise, const double u, const double uNoise);
	~SimpleKalman();

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	void init(const double x, const double y, const double z); // x,y,z : initial joint position
	void getFilteredState(const double x, const double y, const double z, double result[3]); // x,y,z : measured position, result : filtered position vector

private:
	Matrix<double, 6, 6> A, Ex, P, I;
	MatrixXd K;
	Matrix<double, 6, 1> B, estimateState;
	Matrix<double, 3, 6> C;
	Matrix<double, 3, 3> Ez;

	double dt, sensorNoise, u, uNoise;

	void predict(); // no param here, command is supposed to be 0
	void update(const double x, const double y, const double z); // x,y,z : measured position

};

