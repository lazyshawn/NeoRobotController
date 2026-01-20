#pragma once

#include "track_utility.h"

#ifdef __cplusplus
extern "C" {
#endif

// 滤波器
typedef struct FIRFilter {
	//! 输入缓存, 输出缓存
	Queue *xt, *yt;
	//! 滤波器系数
	double *ak, *bk;
	//! 滤波器阶数
	int order;
}FIRFilter, * pFIRFilter;


// 创建滤波器
FIRFilter *firfilter_construct(double* param, int size);

// 销毁滤波器
void firfilter_deconstruct(FIRFilter *q);

// 滤波器重置
void firfilter_clear(FIRFilter *q);

// 单次滤波
double firfilter_process(FIRFilter *q, double sample);

#ifdef __cplusplus
}
#endif
