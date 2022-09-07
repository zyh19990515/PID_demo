#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(float my_A, float my_C, float my_P_, float my_P, float my_Q, float my_R, float my_x_) {
	A = my_A;
	C = my_C;
	P_ = my_P_;
	P = my_P;
	Q = my_Q;
	R = my_R;
	x_ = my_x_;
}

float KalmanFilter::estimate(float zk) {
	float x;
	P_ = A * P * A + Q;
	K = (P_ * C) / ((C * P_ * C) + R);
	x = x_ + K*(zk - C * x_);
	P = (1 - K * C) * P_;
	x_ = x;
	return x;
}