#pragma once

#include "controller_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

// 循环数组实现队列
typedef struct Queue {
	//! 队列数据
	double *data;
	//! 队首，队尾编号
	int begin, end;
	//! 队列最大长度
	int capacity;
}Queue, * pQueue;

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


// 滤波器
typedef struct FIRFilter {
	//! 输入缓存, 输出缓存
	Queue *xt, *yt;
	//! 滤波器系数
	double *ak, *bk;
	//! 滤波器阶数
	int order;
}FIRFilter, * pFIRFilter;


// 创建滤波器
FIRFilter *firfilter_construct(double* param, int size);

// 销毁滤波器
void firfilter_deconstruct(FIRFilter *q);

// 滤波器重置
void firfilter_clear(FIRFilter *q);

// 单次滤波
double firfilter_process(FIRFilter *q, double sample);

#ifdef __cplusplus
}
#endif
