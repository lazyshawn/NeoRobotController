
#ifdef _MSC_VER
#include "data_process/fir_filter.h"
#else
#include "fir_filter.h"
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


/***********************************************************************
 *                        F I R F I L T E R                            *
 ***********************************************************************/
 // 创建滤波器
FIRFilter *firfilter_construct(double* param, int size) {
	FIRFilter *q = (FIRFilter *)malloc(sizeof(FIRFilter));
	// 检查内存是否成功分配
	if (!q) {
		return q;
	}

	int type = (int)param[0];
	switch (type) {
		// 滑动平均滤波
	case 0: {
		// 窗口宽度
		q->order = (int)param[1] - 1;
		q->ak = (double *)malloc(sizeof(double) * (q->order + 1));
		q->bk = (double *)malloc(sizeof(double) * (q->order + 1));
		for (int i = 0; i <= q->order; ++i) {
			q->ak[i] = 0.0;
			q->bk[i] = 1.0 / (q->order + 1);
		}
		q->ak[0] = 1.0;
		break;
	}
			// 巴特沃斯滤波(当前仅支持二阶)
	case 1: {
		q->order = 2;
		double fd = param[2], fs = param[3];
		q->ak = (double *)malloc(sizeof(double) * (q->order + 1));
		q->bk = (double *)malloc(sizeof(double) * (q->order + 1));
		for (int i = 0; i <= q->order; ++i) {
			q->ak[i] = 1.0;
			q->bk[i] = 1.0;
		}

		double w = tan(M_PI * fd / fs);
		double c = 1.0 + sqrt(2.0)*w + w * w;
		q->ak[0] = 1.0;
		q->ak[1] = (2 * w*w - 2.0) / c;
		q->ak[2] = (1.0 - sqrt(2.0)*w + w * w) / c;
		q->bk[0] = w * w / c;
		q->bk[1] = 2.0 * q->bk[0];
		q->bk[2] = q->bk[0];

		break;
	}
			// 不支持
	default:
		break;
	}

	q->xt = queue_construct(q->order);
	q->yt = queue_construct(q->order);

	firfilter_clear(q);

	return q;
}

// 销毁滤波器
void firfilter_deconstruct(FIRFilter *q) {
	if (!q)
		return;

	free(q->ak);
	free(q->bk);
	queue_deconstruct(q->xt);
	queue_deconstruct(q->yt);

	free(q);
}

// 滤波器重置
void firfilter_clear(FIRFilter *q) {
	queue_clear(q->xt);
	queue_clear(q->yt);
}

// 单次滤波
double firfilter_process(FIRFilter *q, double sample) {
	// 滤波器未初始化，返回采样值
	if (!q)
		return sample;

	double ans = q->bk[0] * sample;

	// 采集个数不够, 不滤波
	if (queue_size(q->xt) < q->order) {
		ans = sample;
	}
	// 正常滤波
	else {
		for (int i = 1; i <= q->order; ++i) {
			// bk * x[t-1] - ak * y[t-1]
			ans += q->bk[i] * queue_at(q->xt, q->order - i) - q->ak[i] * queue_at(q->yt, q->order - i);
		}

		ans /= q->ak[0];
	}

	queue_push_back(q->xt, sample);
	queue_push_back(q->yt, ans);

	return ans;
}
