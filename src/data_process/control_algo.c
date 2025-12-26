
#ifdef _MSC_VER
#include "data_process/control_algo.h"
#else
#include "control_algo.h"
#endif

// SMC 构造函数
ControlSMC *smc_construct(double lamb, double eps, double ve0) {
	ControlSMC *q = (ControlSMC *)malloc(sizeof(ControlSMC));
	q->error = 0.0;
	q->lambda = lamb;
	q->epsilon = eps;
	
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
double smc_process(ControlSMC* q, double err) {
	// 误差速率
	double ve = err - q->error;
	// 滑模面计算: S = ve + lamb*e
	double S = ve + q->lambda*err;

	// 计算控制输出: u = -k*ve - a*sgn(S)
	double ans = -q->lambda * ve;

	//double tEps = 1e-2;
	//if (fabs(S) > tEps) {
	//	ans += (S > 0) ? -q->epsilon : q->epsilon;
	//}
	
	return ans;
}
