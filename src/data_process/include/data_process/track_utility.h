#pragma once

/***********************************************************************
 * 电弧跟踪算法通用头文件                                              *
 * 																	   *
 * 定义了常用数据结构                                                  *
 * 定义了常用数学公式，如向量和矩阵计算                                *
 * 定义了控制卡中定义的通用函数                                        *
 ***********************************************************************/

#include "controller_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************************
 *                        Q U E U E                                    *
 ***********************************************************************/
 // 循环数组实现队列
typedef struct Queue {
	//! 队列数据
	double *data;
	//! 队首，队尾编号
	int begin, end;
	//! 队列最大长度
	int capacity;
}Queue, *pQueue;

// 创建队列
Queue *queue_construct(int capacity);

// 销毁队列
void queue_deconstruct(Queue* q);

// 入队, 队列满时自动出队
void queue_push_back(Queue *q, double value);

// 出队
double queue_pop_front(Queue *q);

// 随机访问
double queue_at(Queue *q, int idx);

// 获取队列长度
int queue_size(Queue *q);

// 队列为空
bool queue_empty(Queue *q);

// 清空队列
void queue_clear(Queue *q);



/***********************************************************************
 *                        M A T H                                      *
 ***********************************************************************/
// 向量单位化
int vector_norm(double *vec);

// 向量叉乘
int vector_cross(double *va, double *vb, double *ans);

// 欧拉角转旋转矩阵
int euler2mat(double *euler, double *seq, double *mat);

// 矩阵乘法
int matrix_multiply_in_vector(double *matA, int row, int col, double *matB, int colB, double *ans);


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

// 矩阵析构
void matrix_delete(MatrixXd * q);

// 矩阵格式化输出
int matrix_cout(const MatrixXd * q);

// 矩阵拷贝
MatrixXd * matrix_copy(const MatrixXd * q);

// 获取元素
int matrix_get(const MatrixXd *mat, int row, int col, double *ans);

// 设置元素
int matrix_set(MatrixXd *mat, int row, int col, double val);

// 获取矩阵块
int matrix_get_block(const MatrixXd *matA, int row, int col, MatrixXd *matB);

// 设置矩阵块
int matrix_set_block(MatrixXd *matA, int row, int col, const MatrixXd *matB);

// 矩阵行列式

// 矩阵乘法
int matrix_multiply(const MatrixXd *matA, const MatrixXd *matB, MatrixXd *ans);

// 矩阵求逆
int matrix_LUP_inverse(const MatrixXd* A, MatrixXd* A_inv);

#ifdef __cplusplus
}
#endif
