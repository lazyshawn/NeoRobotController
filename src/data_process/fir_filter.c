
#ifdef _MSC_VER
#include "data_process/fir_filter.h"
#else
#include "fir_filter.h"
#endif


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
			// 不滤波
	case 2: {
		q->order = 0;
	}
			// 不支持
	default:
		break;
	}

	q->xt = queue_construct(q->order);
	q->yt = queue_construct(q->order);
	q->mean = q->var = 0.0;

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

	if (q->order < 1) {
		return sample;
	}

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

// 计算历史输入均值、方差
int firfilter_update_statistical(FIRFilter *q) {
	// 采集个数不够
	if (queue_size(q->xt) < q->order || q->order <= 1) {
		return 1;
	}

	// 均值
	double sum = 0.0;
	for (int i = 0; i < q->order; ++i) {
		sum += queue_at(q->xt, i);
	}
	q->mean = sum / q->order;

	// 标准差
	sum = 0.0;
	for (int i = 0; i < q->order; ++i) {
		double tmp = queue_at(q->xt, i);
		sum += (tmp - q->mean) * (tmp - q->mean);
	}
	q->var = sqrt(sum / (q->order - 1));
	return 0;
}

// 异常值判断
double firfilter_error_check(FIRFilter *q, double sample) {
	int type = 0;
	// 采集个数不够
	if (queue_size(q->xt) < q->order || q->order <= 1) {
		return sample;
	}
	double ans = sample;

	// 1. 均方差异常判断
	double k = 2;
	if (sample > q->mean + k * q->var)
		ans = q->mean + k * q->var;
	else if (sample < q->mean - k * q->var)
		ans = q->mean - k * q->var;

	return ans;
}
