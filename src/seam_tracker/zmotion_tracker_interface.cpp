
#include "zmotion_tracker_interface.h"

#include <iostream>

struct MW_Filter mwFilter;
struct BW_Filter bwFilter;

struct TrackerInfo trackInfo;

int reset_mw_filter(int idx, TYPE_TABLE* addr) {
	if (idx >= filterNum) {
		idx = filterNum - 1;
	}
	else if (idx < 0) {
		idx = 0;
	}

	mwFilter.num[idx] = 0;
	mwFilter.table[idx] = addr;
	mwFilter.saveLen[idx] = 4999;
	mwFilter.windowLen[idx] = 10;
	mwFilter.windowSum[idx] = 0.0;

	return 0;
}

double moving_window_filter(int idx, double sample) {
	double ret = 0.0;
	// 初始数据
	if (mwFilter.num[idx] < mwFilter.windowLen[idx]) {
		mwFilter.windowSum[idx] += sample;
		ret = mwFilter.windowSum[idx] / (mwFilter.num[idx] + 1);
	}
	// 开始滤波
	else {
		mwFilter.windowSum[idx] += sample - mwFilter.table[idx][(mwFilter.num[idx] - mwFilter.windowLen[idx]) % mwFilter.saveLen[idx]];
		ret = mwFilter.windowSum[idx] / mwFilter.windowLen[idx];
	}
	mwFilter.num[idx]++;
	return ret;
}

int reset_butter_worf_filter() {
	return 0;
}
int butter_worf_filter(double sample) {
	return 0;
}


int calc_compensate(int idx, double dArl, double dAud, TYPE_TABLE* output) {
	//output[0] = dArl * trackInfo.rlKp[idx];
	printf("recive: %f, %f.\n", dArl, dAud);
	return 0;
}


int init_filter_tracker(int idx, TYPE_TABLE *config, TYPE_TABLE *data) {
	mwFilter.windowLen[idx] = (int)config[0];
	mwFilter.saveLen[idx] = (int)config[1];
	mwFilter.table[idx] = data;

	trackInfo.rlKp[idx] = config[10];

	trackInfo.udKp[idx] = config[20];

	printf("R%d: %d, %d, %f, %f\n", idx, mwFilter.windowLen[idx], mwFilter.saveLen[idx], trackInfo.rlKp[idx], trackInfo.udKp[idx]);

	return 0;
}
