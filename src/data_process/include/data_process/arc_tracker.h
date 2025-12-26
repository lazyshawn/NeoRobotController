#pragma once

#include "fir_filter.h"
#include "control_algo.h"

// 全局滤波器
#define MaxFilterNum 4

#ifdef __cplusplus
extern "C" {
#endif

// 打印版本信息
void print_release_info();

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

// 向量单位化
int vector_norm(double *vec);

// 向量叉乘
int vector_cross(double *va, double *vb, double *ans);

// 欧拉角转旋转矩阵
int euler2mat(double *euler, double *seq, double *mat);

// 矩阵乘法
int matrix_multiply(double *matA, int row, int col, double *matB, int colB, double *ans);

#ifdef __cplusplus
}
#endif
