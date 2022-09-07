#ifndef KalmanFilter_h
#define KalmanFilter_h


class KalmanFilter
{
public:
	float A;//状态空间方程系数，原方程xk=Axk-1+Buk-1+wk，因系统简单，没有输入，因此B=0，w=0
	float P_;//先验概率分布
	float P;//后验概率分布
	float Q;//方差
	float x_;//先验估计值
	float C;//观测方程系数
	float R;//观测噪声误差
	float K;
	KalmanFilter(float, float, float, float, float, float, float my_x_);
	float estimate(float new_x_k_1);
};

#endif // !KalmanFilter_h