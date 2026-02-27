#pragma once
/* ************************************************************ *
* @brief 矩阵运算辅助库                                         *
*															    *
* 主要功能如下：											    *
* 1. 提供矩阵类，处理矩阵与向量运算                  	        *
* 2. 矩阵LUP分解与求逆											*
* ************************************************************* */


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif // !M_PI

#ifdef _MSC_VER
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#else
// #include "zmcbuildin.h"
#define NULL ((void *)0)
#endif

#ifdef __cplusplus
extern "C" {
#endif

	/***********************************************************************
	 *                        M A T R I X                                  *
	 ***********************************************************************/
	// 矩阵数据结构
	typedef struct {
		int rows;
		int cols;
		// 按行主序存储
		double* data;
	} MatrixXd;

	// 矩阵初始化
	MatrixXd *matrix_new(int rows, int cols, double val);
	MatrixXd *matrix_new_identity(int rows);
	/**
	* @brief  从数组初始化矩阵
	* @param  rows    矩阵行数
	* @param  cols    矩阵列数
	* @param  val     初始化数组
	* @param  num     使用num个数组元素初始化，不足的用0补全. -1: 全部使用数组元素初始化
	* @return 矩阵指针
	*/
	MatrixXd *matrix_from_array(int rows, int cols, const double *val, int num);

	// 矩阵析构
	void matrix_delete(MatrixXd * q);

	// 矩阵拷贝
	MatrixXd * matrix_copy(const MatrixXd * q);

	// 矩阵格式化输出
	int matrix_cout(const MatrixXd * q);
	int matrix_to_array(const MatrixXd * q, double *val, int num);

	// 获取元素
	int matrix_get(const MatrixXd *mat, int row, int col, double *ans);
	double matrix_at(const MatrixXd *mat, int row, int col);

	// 设置元素
	int matrix_set(MatrixXd *mat, int row, int col, double val);

	// 获取矩阵块
	int matrix_get_block(const MatrixXd *matA, int row, int col, MatrixXd *matB);

	// 设置矩阵块
	int matrix_set_block(MatrixXd *matA, int row, int col, const MatrixXd *matB);

	// --- 矩阵校验
	// 矩阵大小校验
	int matrix_same_size(MatrixXd *mat1, MatrixXd *mat2);

	// --- 基础矩阵运算
	// 矩阵行列式

	// 矩阵乘常量
	int matrix_scale(MatrixXd *q, double scale);
	
	// 矩阵加减法
	int matrix_plus(double k1, MatrixXd *mat1, double k2, MatrixXd *mat2, MatrixXd *ans);

	// 矩阵乘法
	int matrix_multiply(const MatrixXd *matA, const MatrixXd *matB, MatrixXd *ans);

	// 矩阵求逆
	int matrix_LUP_inverse(const MatrixXd* A, MatrixXd* A_inv);

#ifdef __cplusplus
}
#endif
