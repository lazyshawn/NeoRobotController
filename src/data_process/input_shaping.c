
#include "data_process/input_shaping.h"

#include "data_process/fir_filter.h"
#include "AuxMatrix.h"

#define COEFF_LEN 2048
// 滤波器系数
static double coeff[MaxAxisNum][COEFF_LEN];
// 历史输入
static double input[MaxAxisNum][COEFF_LEN];

// 使用结构体管理，不直接操作全局变量
static Queue xt[MaxAxisNum];
static FIRFilter filter[MaxAxisNum];

// 加载整形器默认参数
int reset_input_shaping_filter(int num) {
	for (int i = 0; i < MaxAxisNum; ++i) {
		memset(coeff[i], 0, sizeof(double) * COEFF_LEN);

		xt[i].begin = -1;
		xt[i].end = 0;
		xt[i].data = input[i];
		xt[i].capacity = num;

		filter[i].order = num;
		filter[i].xt = &xt[i];
		filter[i].ak = coeff[i];
	}
	return 0;
}

// ZVD 整形, wn: 截止频率, zeta: 阻尼比, T: 时间常数, Ts: 采样周期
int construct_input_shaping_filter(double wn, double zeta, double T, double Ts) {
	double wd = wn * sqrt(1 - zeta * zeta);
	double K = exp(-zeta*M_PI / sqrt(1-zeta*zeta));
	double den = 1.0 + 2 * K + K * K;
	double A1 = 1 / den, A2 = 2 * K / den, A3 = K * K / den;
	double tau = M_PI / wd;
	// 延迟采样点数
	int d = ceil(tau / Ts);

	// 滤波器结构体初始化
	reset_input_shaping_filter(2 * d + 1);

	for (int i = 0; i < MaxAxisNum; ++i) {
		coeff[i][0] = A1;
		coeff[i][d] = A2;
		coeff[i][2 * d] = A3;
	}

	return 0;
}


// 低通滤波整形，消除一阶模态 N=1, 平滑阶数 M=2
int construct_low_pass_input_shaping(double wn, double zeta, double T, double Ts) {
	int K_fir = ceil(T / Ts);
	double sigma = zeta * wn;

	MatrixXd *c = matrix_new(5, 1, 0.0);
	MatrixXd *matT = matrix_new(K_fir + 1, 5, 0.0);
	MatrixXd *matTt = matrix_new(5, K_fir + 1, 0.0);
	MatrixXd *TtT = matrix_new(5, 5, 0.0);
	MatrixXd *LSM = matrix_new(K_fir + 1, 5, 0.0);
	MatrixXd *q = matrix_new(K_fir + 1, 1, 0.0);

	for (int i = 0; i < K_fir + 1; ++i) {
		double t = i * Ts;

		double tmp = exp(sigma*t)*cos(wn*t);
		matrix_set(matT, i, 0, tmp);
		matrix_set(matTt, 0, i, tmp);
		tmp = exp(sigma*t)*sin(wn*t);
		matrix_set(matT, i, 1, tmp);
		matrix_set(matTt, 1, i, tmp);

		matrix_set(matT, i, 2, 1.0);
		matrix_set(matTt, 2, i, 1.0);
		matrix_set(matT, i, 3, -i);
		matrix_set(matTt, 3, i, -i);
		matrix_set(matT, i, 4, i*i / 2.0);
		matrix_set(matTt, 4, i, i*i / 2.0);
	}
	matrix_set(c, 4, 0, 1.0);

	// 最小范数解 q = matT (matT^T matT)^-1 c
	matrix_multiply(matTt, matT, TtT);
	matrix_LUP_inverse(TtT, TtT);
	matrix_multiply(matT, TtT, LSM);
	matrix_multiply(LSM, c, q);

	// 滤波器结构体初始化
	reset_input_shaping_filter(K_fir + 1);

	// 积分得到滤波器系数
	for (int j = 0; j < MaxAxisNum; ++j) {
		for (int k = 0; k < K_fir + 1; ++k) {
			coeff[j][k] = 0.0;
			for (int i = 0; i <= k; ++i) {
				double tmp;
				matrix_get(q, k - i, 0, &tmp);
				coeff[j][k] += (i + 1) * tmp;
			}
		}
	}

	matrix_delete(c);
	matrix_delete(matT);
	matrix_delete(matTt);
	matrix_delete(TtT);
	matrix_delete(LSM);
	matrix_delete(q);
	return 0;
}


// 输入整形滤波器
double input_shaping_filter_onesample(double sample) {
	double ans = 0.0;
	int idx = queue_size(filter[0].xt);

	// 第一次调用时初始化，用初始值填满缓冲区
	if (idx < 1) {
		for (int i = 0; i < filter[0].order; ++i) {
			queue_push_back(filter[0].xt, sample);
		}
	}
	// 正常存入新样本
	else {
		queue_push_back(filter[0].xt, sample);
	}

	for (int i = 0; i < filter[0].order; ++i) {
		int readIdx = (filter[0].xt->end-1 - i) % filter[0].order;
		if (readIdx < 0) {
			readIdx += filter[0].order;
		}
		ans += filter[0].ak[i] * filter[0].xt->data[readIdx];
	}

	return ans;
}


int input_shaping_filter_onestep(double *JPosIn, double *JPosOut, int num) {
	int shapeNum = MaxAxisNum, maxNum = num;
	if (num < MaxAxisNum) {
		shapeNum = num;
		maxNum = num;
	}

	for (int k = 0; k < shapeNum; ++k) {
		double ans = 0.0;
		int idx = queue_size(filter[k].xt);
		// 第一次调用时初始化，用初始值填满缓冲区
		if (idx < 1) {
			for (int i = 0; i < filter[k].order; ++i) {
				queue_push_back(filter[k].xt, JPosIn[k]);
			}
		}
		// 正常存入新样本
		else {
			queue_push_back(filter[k].xt, JPosIn[k]);
		}

		for (int i = 0; i < filter[k].order; ++i) {
			int readIdx = (filter[k].xt->end - 1 - i) % filter[k].order;
			if (readIdx < 0) {
				readIdx += filter[k].order;
			}
			ans += filter[k].ak[i] * filter[k].xt->data[readIdx];
		}
		JPosOut[k] = ans;
	}

	for (int k = shapeNum; k < maxNum; ++k) {
		JPosOut[k] = JPosIn[k];
	}

	return 0;
}
