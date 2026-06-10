#pragma once

#include "controller_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
* @brief  单例卡尔曼滤波器初始化，状态转移矩阵和观测矩阵默认使用单位矩阵，输入矩阵默认使用零矩阵
* @param  mat     矩阵指针，不要求初始化
* @param  rows    矩阵行数
* @param  cols    矩阵列数
* @param  val     初始化数组
*/
SHARE_API_ int kalman_filter_init(int numState, int numObs, int numIn,
	const double* covQ, const double* covR,
	const double* Pk0, const double* x0);

SHARE_API_ int kalman_filter_process(double* input, double* output);


#ifdef __cplusplus
}
#endif