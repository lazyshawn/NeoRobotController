
#if defined(_WIN32) && defined(_MSC_VER)
#include "data_process/track_utility.h"
#elif defined(__linux__) && defined(__GNUC__)
#include "data_process/track_utility.h"
#else
#include "track_utility.h"
#endif


/***********************************************************************
 *                        Z M O T I O N                                *
 ***********************************************************************/
#if !defined(_MSC_VER) && !defined(__linux__)
 // 获取轴脉冲值
int32 get_axis_pulse(int mode, uint32 iaxis) {
	if (mode == 1) {
		return motionrt_getpuldpos(iaxis);
	}
	else {
		return motionrt_getpulmpos(iaxis);
	}
}
#endif


/***********************************************************************
 *                        Q U E U E                                    *
 ***********************************************************************/
// 创建队列
Queue *queue_construct(int capacity) {
	Queue *q = (Queue *)malloc(sizeof(Queue));
	q->data = (double *)malloc(sizeof(double) * capacity);
	q->begin = -1;
	q->end = 0;
	q->capacity = capacity;
	return q;
}

// 销毁队列
void queue_deconstruct(Queue* q) {
	free(q->data);

	free(q);
}

// 入队
void queue_push_back(Queue *q, double value) {
	// 队满
	if (queue_size(q) == q->capacity)
		queue_pop_front(q);
	// 队空
	else if (queue_size(q) == 0)
		q->begin = 0;

	q->data[q->end] = value;
	// 回绕
	q->end = (q->end + 1) % q->capacity;
}

// 出队
double queue_pop_front(Queue *q) {
	// 队空
	if (queue_size(q) == 0) {
		q->begin = -1;
		return 0.0;
	}

	double value = q->data[q->begin];
	q->begin = (q->begin + 1) % q->capacity;

	return value;
}

// 随机访问
double queue_at(Queue *q, int idx) {
	return q->data[(q->begin + idx) % q->capacity];
}

// 获取队列长度
int queue_size(Queue *q) {
	if (q->begin < 0)
		return 0;

	return (q->end > q->begin) ? (q->end - q->begin) : (q->capacity + q->end - q->begin);
}

// 队列为空
bool queue_empty(Queue *q) {
	return queue_size(q) == 0;
}

// 清空队列
void queue_clear(Queue *q) {
	q->begin = -1;
	q->end = 0;
}
