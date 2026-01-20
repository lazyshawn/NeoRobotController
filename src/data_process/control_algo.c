
#ifdef _MSC_VER
#include "data_process/control_algo.h"
#else
#include "control_algo.h"
#endif


/***********************************************************************
*                        Sliding Mode Control                          *
* **********************************************************************/
// SMC 构造函数
ControlSMC *smc_construct(double lamb, double eps, double ve0, double ks) {
	ControlSMC *q = (ControlSMC *)malloc(sizeof(ControlSMC));
	q->error = 0.0;
	q->lambda = lamb;
	q->epsilon = eps;
	q->ks = ks;
	
	return q;
}

// SMC 析构函数
void smc_deconstruct(ControlSMC *q) {
	free(q);
}

// SMC 重置
void smc_clear(ControlSMC *q) {
	q->error = 0.0;
}

// 根据当前误差计算控制输出
double smc_process(ControlSMC* q, double err, double gain) {
	// 误差速率
	double ve = err - q->error;
	// 滑模面计算: S = ve + lamb*e
	double S = ve + q->lambda * err;
	
	// 计算控制输出: u = -a*ve -b*S -c*sgn(S)
	double ans = -q->lambda * ve - q->ks*S - (1 - exp(-S * 1.0)) /(1 + exp(-S * 1.0))*q->epsilon;
	//double ans = -q->lambda * ve * gain - q->ks*S;
	//if (fabs(S) > 1e-2) {
	//	ans += (S > 0) ? -q->epsilon : q->epsilon;
	//}
	printf("smc: ve = %f, S = %f\n", ve, S);

	// 更新历史误差
	q->error = err;
	
	return ans;
}


/***********************************************************************
*                        Bias Estimator                                *
* **********************************************************************/
// BE 构造函数
BiasEstimator *bias_estimator_construct(int order) {
	BiasEstimator *q = (BiasEstimator *)malloc(sizeof(BiasEstimator));
	return q;
}

// BE 析构函数
void *bias_estimator_deconstruct(BiasEstimator *q) {
}

// BE 初始化
int *bias_estimator_clear(BiasEstimator *q) {
	return 0;
}

/**
* 根据当前误差计算控制输出
* @param  *q       WA 指针
* @param  data     待处理数据
* @return 控制输出
*/
double bias_estimator_process(BiasEstimator* q, double data) {
	return 0;
}
