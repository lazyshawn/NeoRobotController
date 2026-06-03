#pragma once
/* ************************************************************ *
* @brief 电弧跟踪补偿控制器                                     *
*															    *
* 主要功能如下：											    *
* 1. 估计左右偏差方向和偏差值    			                    *
* 2. 估计上下偏差方向和偏差值                   		        *
* ************************************************************* */

#include "track_utility.h"


#ifdef __cplusplus
extern "C" {
#endif


/***********************************************************************
*                        Sliding Mode Control                          *
* **********************************************************************/
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
* @param  err      当前误差
* @param  gain     增益系数
* @return 控制输出
*/
double smc_process(ControlSMC* q, double err, double gain);


/***********************************************************************
*                        Bias Estimator                                *
* **********************************************************************/
// 偏差估计器
typedef struct BiasEstimator {
	//! 历史数据个数
	int order;
	//! 历史误差
	Queue *xt;
	//! 加权系数
	double *kp;
}BiasEstimator, *pBiasEstimator;

// BE 构造函数
BiasEstimator *bias_estimator_construct(int order);

// BE 析构函数
void bias_estimator_deconstruct(BiasEstimator *q);

// BE 初始化
int *bias_estimator_clear(BiasEstimator *q);

/**
* 根据当前误差计算控制输出
* @param  *q       WA 指针
* @param  data     待处理数据
* @return 控制输出
*/
double bias_estimator_process(BiasEstimator* q, double data);

#ifdef __cplusplus
}
#endif
