#pragma once
/***********************************************************************
 * 电弧跟踪算法顶层头文件                                              *
 * 																	   *
 * 电弧跟踪算法库共包括三部分：滤波、周期参考值计算、偏差值估计        *
 * 该头文件包含了提供给上层的跟踪算法接口，接口包含以下部分：          *
 * 1. 滤波器的构造、析构、初始化和单次数据处理                         *
 * 2. 计算样本区间参考值											   *
 * 3. 补偿值计算。补偿方向和补偿量，进一步的结果为机器人坐标系下的偏移 *
 ***********************************************************************/

#include "fir_filter.h"
#include "control_algo.h"

// 全局滤波器
#define MaxFilterNum 4

#ifdef __cplusplus
extern "C" {
#endif

// 构造滤波器
void filter_construct(int idx, double* param, int num);

// 销毁滤波器
void filter_deconstruct(int idx);

// 重置滤波器
void filter_clear(int idx);

// 单次滤波
double filter_process(int idx, double sample);

// 计算样本区间参考值
double calc_interval_refrence(double *config, double *data);

// 计算补偿量, 估计偏移量/误差
int calc_compensate(int idx, double *config, double *data);

// 计算控制量

#ifdef __cplusplus
}
#endif
