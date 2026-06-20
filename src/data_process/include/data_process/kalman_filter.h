#pragma once

#include "controller_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

// 力传感器标定: 根据姿态序列与对应的力传感器测量值，计算传感器零点偏移和标定矩阵
SHARE_API_ int force_sensor_calibration(double* input, double* output);

// 力传感器负载辨识: 根据姿态序列与对应的力传感器测量值，计算负载质量和重心位置
SHARE_API_ int force_sensor_identification(double* input, double* output);

// 力传感器初始化: 初始化传感器参数，后续可以增加传感器自检功能
SHARE_API_ int force_sensor_init(const double* rs, const double* gs,
	const double* covQ, const double* covR, const double* Pk0, const double* x0);

// 力传感器补偿: 根据当前姿态与测量值，计算传感器实际力值，补偿掉重力引起的误差，后续可以增加惯性力、科氏力等补偿功能
SHARE_API_ int force_sensor_compensation(const double euler[3], const double input[6], double output[6]);

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