#pragma once

#include "track_utility.h"

#ifdef __cplusplus
extern "C" {
#endif

// 滤波器
typedef struct SHARE_API_ FIRFilter {
	//! 输入缓存, 输出缓存
	Queue *xt, *yt;
	//! 滤波器系数
	double *ak, *bk;
	//! 滤波器阶数
	int order;
	//! 缓存数据均值，方差
	double mean, var;
}FIRFilter, * pFIRFilter;


// 创建滤波器
SHARE_API_ FIRFilter *firfilter_construct(double* param, int size);

// 销毁滤波器
SHARE_API_ void firfilter_deconstruct(FIRFilter *q);

// 滤波器重置
SHARE_API_ void firfilter_clear(FIRFilter *q);

// 单次滤波
SHARE_API_ double firfilter_process(FIRFilter *q, double sample);

// 计算历史输入均值、方差
SHARE_API_ int firfilter_update_statistical(FIRFilter *q);

// 异常值判断
SHARE_API_ double firfilter_error_check(FIRFilter *q, double sample);

#ifdef __cplusplus
}
#endif
