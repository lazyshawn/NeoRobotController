#pragma once

#ifdef __cplusplus
extern "C" {
const int filterNum = 2;
#else

#define filterNum 2

#include "zmcbuildin.h"
#include "motionrt_file3_cfunc.h"
#include "rthmi_cfunc.h"

#endif

typedef double TYPE_TABLE;

struct MW_Filter {
	// 窗口宽度
	int windowLen[filterNum];
	// 窗口数据总和
	double windowSum[filterNum];
	// 已完成的滤波次数
	long num[filterNum];
	// 最大储存长度
	int saveLen[filterNum];
	// 原始数据保存位置
	double* table[filterNum];
};

// 重置滑动平均滤波器
int reset_mw_filter(int idx, TYPE_TABLE* addr);
// 滑动平均滤波
double moving_window_filter(int idx, double sample);


struct BW_Filter {
};

int reset_butter_worf_filter();
int butter_worf_filter(double sample);


// 跟踪参数
struct TrackerInfo {
	// 左右跟踪
	double rlKp[filterNum];
	double rlKi[filterNum];
	double rlKd[filterNum];

	// 上下跟踪
	double udKp[filterNum];
	double udKi[filterNum];
	double udKd[filterNum];
};

// 计算偏移量
int calc_compensate(int idx, double dArl, double dAud, TYPE_TABLE* output);


// 读取滤波器和跟踪的配置参数
int init_filter_tracker(int idx, TYPE_TABLE *config, TYPE_TABLE *data);

#ifdef __cplusplus
}
#endif
