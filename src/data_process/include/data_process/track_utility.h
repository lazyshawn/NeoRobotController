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
typedef struct SHARE_API_ Queue {
	//! 队列数据
	double *data;
	//! 队首，队尾编号
	int begin, end;
	//! 队列最大长度
	int capacity;
}Queue, *pQueue;

// 创建队列
SHARE_API_ Queue *queue_construct(int capacity);

// 销毁队列
SHARE_API_ void queue_deconstruct(Queue* q);

// 入队, 队列满时自动出队
SHARE_API_ void queue_push_back(Queue *q, double value);

// 出队
SHARE_API_ double queue_pop_front(Queue *q);

// 随机访问
SHARE_API_ double queue_at(Queue *q, int idx);

// 获取队列长度
SHARE_API_ int queue_size(Queue *q);

// 队列为空
SHARE_API_ bool queue_empty(Queue *q);

// 清空队列
SHARE_API_ void queue_clear(Queue *q);

#ifdef __cplusplus
}
#endif
