#pragma once

#ifndef MaxAxisNum
#define MaxAxisNum 9
#endif // !MaxAxisNum

#include "controller_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 构造ZVD整形器
 * 
 * @param wn    截止频率
 * @param zeta  阻尼比
 * @param T    时间常数(无效)
 * @param Ts   采样周期
 * @return int 0: 成功, -1: 失败
 */
SHARE_API_ int construct_input_shaping_filter(double wn, double zeta, double T, double Ts);

/**
 * @brief 构造低通整形器
 * 
 * @param wn    截止频率
 * @param zeta  阻尼比
 * @param T     时间常数, 阶跃响应稳定时间
 * @param Ts    采样周期
 * @return int 0: 成功, -1: 失败
 */
SHARE_API_ int construct_low_pass_input_shaping(double wn, double zeta, double T, double Ts);

/**
 * @brief 单步输入整形
 * @param JPosIn  输入指令数组
 * @param JPosOut 整形指令数组
 * @param num     轴数
 * @return int 0: 成功, -1: 失败
 */
SHARE_API_ int input_shaping_filter_onestep(double *JPosIn, double *JPosOut, int num);

#ifdef __cplusplus
}
#endif