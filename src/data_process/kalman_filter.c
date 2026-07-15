#include "data_process/kalman_filter.h"

#include "AuxMatrix.h"
#include "AuxRotRep.h"


typedef struct KalmanFilter {
	// 状态Xk维度，输入Uk维度，观测Zk维度
	int m_numState, m_numIn, m_numObs;
	// 状态向量
	double X[6];
	// 观测向量
	double Z[6];
	// 状态转移矩阵
	double F[36];
	double B[6];
	// 观测矩阵
	double H[36];
	// 过程噪声协方差矩阵: wk ~ N(0, Q)
	double Q[36];
	// 观测噪声协方差矩阵: vk ~ N(0, R)
	double R[36];
	// 后验估计误差协方差矩阵
	double P[36];
	// 卡尔曼增益矩阵
	double K[36];
}KalmanFilter, * pKalmanFilter;


// 使用静态变量存储矩阵，避免频繁分配和释放内存，注意**不是**线程安全的设计
static MatrixXd Rk, Qk, Fk, Bk, Pk, Kk, Hk, Xk, Zk;
static KalmanFilter gs_filter;
static MatrixXd mGs, Rc;

int kalman_filter_init(int numState, int numObs, int numIn,
	const double* covQ, const double* covR,
	const double* Pk0, const double* x0) 
{
	memset(&gs_filter, 0, sizeof(KalmanFilter));
	// --- 1. 滤波器初始化
	gs_filter.m_numState = 6;
	gs_filter.m_numObs = 6;
	gs_filter.m_numIn = 1;
	// 状态转移矩阵，过程模型为随机游走 (xkn = xk + wk)
	gs_filter.F[0] = 1; gs_filter.F[7] = 1; gs_filter.F[14] = 1; gs_filter.F[21] = 1; gs_filter.F[28] = 1; gs_filter.F[35] = 1;
	// 观测矩阵，观测值等于真实值加观测噪声 (Zk = Hk Xk + vk)
	gs_filter.H[0] = 1; gs_filter.H[7] = 1; gs_filter.H[14] = 1; gs_filter.H[21] = 1; gs_filter.H[28] = 1; gs_filter.H[35] = 1;

	for (int i = 0; i < numState; ++i) {
		// 模型噪声协方差
		gs_filter.Q[i * numState + i] = covQ[i];
		// 误差协方差初值
		gs_filter.P[i * numState + i] = Pk0[i];
		// 状态初值
		gs_filter.X[i] = x0[i];
	}
	for (int i=0; i < numObs; ++i) {
		// 观测噪声协方差
		gs_filter.R[i * numObs + i] = covR[i];
	}

	// --- 2. 绑定矩阵指针与滤波器参数数组
	matrix_attach(&Qk, gs_filter.m_numState, gs_filter.m_numState, gs_filter.Q);
	matrix_attach(&Rk, gs_filter.m_numObs, gs_filter.m_numObs, gs_filter.R);
	matrix_attach(&Fk, gs_filter.m_numState, gs_filter.m_numState, gs_filter.F);
	matrix_attach(&Bk, gs_filter.m_numState, gs_filter.m_numIn, gs_filter.B);
	matrix_attach(&Pk, gs_filter.m_numState, gs_filter.m_numState, gs_filter.P);
	matrix_attach(&Kk, gs_filter.m_numState, gs_filter.m_numObs, gs_filter.K);
	matrix_attach(&Hk, gs_filter.m_numObs, gs_filter.m_numState, gs_filter.H);
	matrix_attach(&Xk, gs_filter.m_numState, 1, gs_filter.X);
	matrix_attach(&Zk, gs_filter.m_numObs, 1, gs_filter.Z);

	return 0;
}

// 考虑稳态过程，可忽略线加速度和角加速度的情况，等效于多维低通滤波器
int kalman_filter_process(double* input, double* output) {
	// 更新观测值
	matrix_attach(&Zk, gs_filter.m_numObs, 1, input);

	// --- 1. 预测步骤
	// 1.1 当前状态先验估计: Xkn = Fk Xk + B uk + wk
	matrix_multiply(&Fk, &Xk, &Xk);
	matrix_plus(1.0, &Xk, 0.0, &Bk, &Xk);

	// 1.2 先验估计误差协方差: Pkn = Fk Pk Fk^T + Qk
	matrix_multiply(&Fk, &Pk, &Pk);
	matrix_transpose(&Fk);
	matrix_multiply(&Pk, &Fk, &Pk);
	matrix_transpose(&Fk);
	matrix_plus(1.0, &Pk, 1.0, &Qk, &Pk);

	// --- 2. 更新步骤
	// 2.1 计算卡尔曼增益: Kk = Pkn Hk^T (Hk Pkn Hk^T + Rk)^-1
	MatrixXd *PkRk = matrix_new_copy(&Pk);
	MatrixXd *HkT = matrix_new_copy(&Hk);
	matrix_transpose(HkT);

	matrix_multiply(&Hk, &Pk, PkRk);
	matrix_multiply(PkRk, HkT, PkRk);
	matrix_plus(1.0, PkRk, 1.0, &Rk, PkRk);
	matrix_LUP_inverse(PkRk, PkRk);
	matrix_multiply(&Pk, HkT, &Kk);
	matrix_multiply(&Kk, PkRk, &Kk);

	// 2.2 更新状态后验估计: Xkn = Xk + Kk (Zk - Hk Xk)
	MatrixXd *ZXk = matrix_new_copy(&Zk);
	MatrixXd *KXk = matrix_new_copy(&Xk);

	matrix_multiply(&Hk, &Xk, ZXk);
	matrix_plus(1.0, &Zk, -1.0, ZXk, ZXk);
	matrix_multiply(&Kk, ZXk, KXk);
	matrix_plus(1.0, &Xk, 1.0, KXk, &Xk);

	// 2.3 后验估计误差协方差: Pkn = (I - Kk Hk) Pk
	MatrixXd *KHk = matrix_new_copy(&Pk);
	MatrixXd *Ident = matrix_new_identity(gs_filter.m_numState);

	matrix_multiply(&Kk, &Hk, KHk);
	matrix_plus(1.0, Ident, -1.0, KHk, KHk);
	matrix_multiply(KHk, &Pk, &Pk);

	// --- 3. 将更新后的状态估计值写回输出数组
	matrix_to_array(&Xk, output, gs_filter.m_numState);

	// 释放内存
	matrix_delete(PkRk);
	matrix_delete(HkT);
	matrix_delete(ZXk);
	matrix_delete(KXk);
	matrix_delete(KHk);
	matrix_delete(Ident);

	return 0;
}

// 力传感器标定
int force_sensor_calibration(double* input, double* output) {
	return 0;
}

// 力传感器负载辨识
int force_sensor_identification(double* input, double* output) {
	return 0;
}

// 力传感器初始化
int force_sensor_init(const double* rs, const double* gs, 
	const double* covQ, const double* covR, const double* Pk0, const double* x0)
{
	// 负载参数初始化
	matrix_attach(&mGs, 3, 1, gs);
	matrix_attach(&Rc, 3, 1, rs);

	// 滤波器初始化
	kalman_filter_init(6, 6, 1, covQ, covR, Pk0, x0);

	return 0;
}

// 力传感器补偿
int force_sensor_compensation(const double euler[3], const double input[6], double output[6]) {
	// 计算传感器姿态矩阵
	double Rot[9];
	Euler2Rot(euler, Rot);
	MatrixXd R;
	matrix_attach(&R, 3, 3, Rot);
	MatrixXd* RT = matrix_new_copy(&R);
	matrix_transpose(RT);

	// 传感器测量值分解为力和力矩两部分
	MatrixXd Fssr, Tssr;
	matrix_attach(&Fssr, 3, 1, input);
	matrix_attach(&Tssr, 3, 1, &input[3]);

	// 重力补偿
	MatrixXd *Fobs = matrix_new_copy(&mGs);
	MatrixXd* Tobs = matrix_new_copy(&mGs);
	matrix_multiply(RT, &mGs, Fobs);
	matrix_outer_product(&Rc, Fobs, Tobs);

	matrix_plus(1.0, &Fssr, -1.0, Fobs, Fobs);
	matrix_plus(1.0, &Tssr, -1.0, Tobs, Tobs);

	// 卡尔曼滤波
	double obs[6] = { 0.0 };
	matrix_to_array(Fobs, obs, 3);
	matrix_to_array(Tobs, &obs[3], 3);
	kalman_filter_process(obs, output);

	matrix_delete(RT);
	matrix_delete(Fobs);
	matrix_delete(Tobs);

	return 0;
}
