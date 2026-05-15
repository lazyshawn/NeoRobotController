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

#include "common/ExportSharedAPI.h"

#ifdef _MSC_VER
#include <math.h>
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
		// 按行主序存储: 第i行第j列的元素为data[i*cols+j]
		double* data;
	} MatrixXd;

	// 矩阵初始化
	SHARE_API_ MatrixXd *matrix_new(int rows, int cols, double val);
	SHARE_API_ MatrixXd *matrix_new_identity(int rows);
	SHARE_API_ void matrix_set_all(MatrixXd *mat, double val);
	SHARE_API_ void matrix_set_identity(MatrixXd *mat);
	/**
	* @brief  从数组初始化矩阵
	* @param  rows    矩阵行数
	* @param  cols    矩阵列数
	* @param  val     初始化数组
	* @param  num     使用num个数组元素初始化，不足的用0补全. -1: 全部使用数组元素初始化
	* @return 矩阵指针
	*/
	SHARE_API_ MatrixXd *matrix_from_array(int rows, int cols, const double *val, int num);
	SHARE_API_ int matrix_copy_array(MatrixXd *mat, const double *val, int num);

	// 矩阵析构
	SHARE_API_ void matrix_delete(MatrixXd * q);

	// 矩阵拷贝
	SHARE_API_ MatrixXd * matrix_new_copy(const MatrixXd * q);
	SHARE_API_ void matrix_copy(const MatrixXd * q, MatrixXd *ans);

	// 矩阵格式化输出
	SHARE_API_ int matrix_cout(const MatrixXd * q);
	SHARE_API_ int matrix_to_array(const MatrixXd * q, double *val, int num);

	// 获取元素
	SHARE_API_ int matrix_get(const MatrixXd *mat, int row, int col, double *ans);
	SHARE_API_ double matrix_at(const MatrixXd *mat, int row, int col);

	// 设置元素
	SHARE_API_ int matrix_set(MatrixXd *mat, int row, int col, double val);

	// 矩阵转置
	SHARE_API_ int matrix_transpose(MatrixXd *mat);

	// 获取矩阵块
	SHARE_API_ int matrix_get_block(const MatrixXd *matA, int row, int col, MatrixXd *matB);

	// 设置矩阵块
	SHARE_API_ int matrix_set_block(MatrixXd *matA, int row, int col, const MatrixXd *matB);

	// --- 矩阵校验
	// 矩阵大小校验
	SHARE_API_ int matrix_same_size(const MatrixXd *mat1, const MatrixXd *mat2);

	// --- 基础矩阵运算
	// 矩阵行列式

	// 矩阵乘常量
	SHARE_API_ int matrix_scale(MatrixXd *q, double scale);
	
	// 矩阵加减法
	SHARE_API_ int matrix_plus(double k1, const MatrixXd *mat1, double k2, const MatrixXd *mat2, MatrixXd *ans);

	// 矩阵乘法
	SHARE_API_ int matrix_multiply(const MatrixXd *matA, const MatrixXd *matB, MatrixXd *ans);

	// 矩阵范数
	SHARE_API_ double matrix_norm(const MatrixXd *mat);
	SHARE_API_ double matrix_squared_norm(const MatrixXd *mat);

	// 矩阵单位化
	SHARE_API_ int matrix_normalize(MatrixXd *mat);

	// 矩阵内积
	SHARE_API_ double matrix_inner_product(const MatrixXd *matA, const MatrixXd *matB);

	// 矩阵外积
	SHARE_API_ int matrix_outer_product(const MatrixXd *matA, const MatrixXd *matB, MatrixXd *ans);

	// 矩阵求逆
	SHARE_API_ int matrix_LUP_inverse(const MatrixXd* A, MatrixXd* A_inv);

	// --- 部分姿态转换相关的补充函数，后续移动到新的文件中
	// 罗德里格斯公式
	SHARE_API_ int matrix_rodrigues(const MatrixXd *k, const MatrixXd *p, double theta, MatrixXd *ans);

	// 轴角公式
	SHARE_API_ int matrix_axis_angle(const MatrixXd *k, double theta, MatrixXd *R);

#ifdef __cplusplus
}
#endif
