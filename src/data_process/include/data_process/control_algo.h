#pragma once

#include "controller_interface.h"


#ifdef __cplusplus
extern "C" {
#endif

// SMC 控制算法
typedef struct ControlSMC {
	//! 历史误差
	double error;
	//! 滑模面参数
	double lambda, ks;
	//! 趋近律参数
	double epsilon;
}ControlSMC, *pControlSMC;

// SMC 构造函数
ControlSMC *smc_construct(double lamb, double eps, double err0, double ks);

// SMC 析构函数
void smc_deconstruct(ControlSMC *q);

// SMC 重置
void smc_clear(ControlSMC *q);

/**
	* 根据当前误差计算控制输出
	* @param  *q       SCM 指针
	* @param  *err     当前误差
	* @return 控制输出
	*/
double smc_process(ControlSMC* q, double err, double gain);

#ifdef __cplusplus
}
#endif
