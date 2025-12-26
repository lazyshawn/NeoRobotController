
#ifdef _MSC_VER
#include "data_process/arc_tracker.h"
#else
#include "arc_tracker.h"
#endif

// 误差临界值
const double eps = 1e-6;

/***********************************************************************
 *                        Z M O T I O N                                *
 ***********************************************************************/
#ifndef _MSC_VER
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

FIRFilter *filter[MaxFilterNum];
ControlSMC *smc[MaxFilterNum];

void print_release_info() {

}

/***********************************************************************
 *                        F I L T E R                                  *
 ***********************************************************************/
// 定义滤波器
void filter_construct(int idx, double* param, int num) {
	if (idx >= MaxFilterNum || idx < 0)
		return;

	// 滤波器初始化
	filter[idx] = firfilter_construct(param, num);
	printf("%d\tnew filter(%d): ", idx, (int)param[0]);
	for (int i = 1; i < num; ++i) {
		printf("%f, ", param[i]);
	}
	printf("\n");

	// 控制算法初始化
	smc[idx] = smc_construct(1.0, 1.0, 0);
}

// 销毁滤波器
void filter_deconstruct(int idx) {
	if (idx >= MaxFilterNum || idx < 0)
		return;

	firfilter_deconstruct(filter[idx]);
	smc_deconstruct(smc[idx]);
}

// 重置滤波器
void filter_clear(int idx) {
	firfilter_clear(filter[idx]);
	smc_clear(smc[idx]);
}

// 单次滤波
double filter_process(int idx, double sample) {
	return firfilter_process(filter[idx], sample);
}


/***********************************************************************
 *                        T R A C K I N G                              *
 ***********************************************************************/
/**
 * 计算样本区间参考值
 * @param  *config  配置地址
 * @param  *data    数据地址
 */
double calc_interval_refrence(double *config, double *data) {
	// --- config 数组
	// 数据长度
	int maxSampleNum = (int)config[0];
	// 样本区间起止位置(闭区间)
	int begIdx = (int)config[1], endIdx = (int)config[2];
	// 参考值计算方法 + 算法参数
	// 计算结果

	// --- 异常情况处理
	if (begIdx >= endIdx) {
		return data[(begIdx) % maxSampleNum];
	}

	// --- 四分位点
	// 复制区间数组
	int arrSize = endIdx - begIdx + 1;
	double *arr = (double *)malloc(sizeof(double) * (arrSize));
	for (int i = 0; i < arrSize; ++i) {
		arr[i] = data[(begIdx + i) % maxSampleNum];
		//printf("%lf, %lf, %d\n", arr[i], data[(begIdx + i) % maxSampleNum], (begIdx + i) % maxSampleNum);
	}

	// 冒泡排序: 从小到大 [0,0.75]
	int firIdx = arrSize * 0.25;
	int ansIdx = arrSize * 0.75;
	for (int i = 0; i < ansIdx + 1; ++i) {
		for (int j = i; j < arrSize; ++j) {
			if (arr[i] > arr[j]) {
				double tmp = arr[j];
				arr[j] = arr[i];
				arr[i] = tmp;
			}
		}
		//printf("%lf\n", arr[i]);
	}
	//printf("ans = %lf\n", arr[ansIdx]);

	// 求均值
	//double sum = 0.0;
	//for (int i = firIdx; i < ansIdx + 1; ++i) {
	//	sum += arr[i];
	//}

	return arr[ansIdx];
}

/**
 * 计算补偿量
 * @param  *config  配置地址
 * @param  [out] *data    输出结果地址
 */
int calc_compensate(int idx, double* config, double* data) {
	// 临时数组
	double vec[3], mat[9];

	// --- config 数组
	// 跟踪使能
	int enable = (int)config[7];
	// 左右跟踪设置
	int enableRL = (int)config[0];
	double offsetRL = config[1];
	double gainRL = config[2];
	double maxSingleRL = fabs(config[5]);
	// 上下跟踪设置
	int enableUD = (int)config[10];
	double offsetUD = config[11];
	double gainUD = config[12];
	double goalUD = config[17];

	// 参考电流
	double AR = config[31];
	double AL = config[32];
	double ACref = config[46];
	double AC = (AR + AL) / 2;

	// 轨迹切向
	double tanDir[3] = { config[33], config[34], config[35] };
	if (vector_norm(tanDir)) {
		printf("tanDir error: %f, %f, %f\n", tanDir[0], tanDir[1], tanDir[2]);
		return 0;
	}
	// 焊枪方向
	double rx = config[36] * M_PI / 180, ry = config[37] * M_PI / 180, rz = config[38] * M_PI / 180;
	euler2mat((double[]){ rx,ry,rz }, (double[]){ 0,1,2 }, mat);
	double zDir[3] = { mat[2], mat[5], mat[8] };
	// 焊枪方向从基坐标系转到世界坐标系
	rx = config[40] * M_PI / 180;
	ry = config[41] * M_PI / 180;
	rz = config[42] * M_PI / 180;
	euler2mat((double[]) { rx, ry, rz }, (double[]) { 0, 1, 2 }, mat);
	matrix_multiply(mat, 3, 3, zDir, 1, vec);
	memcpy(zDir, vec, 3 * sizeof(double));

	// 主运动距离
	double masterDist = config[48] - config[47];
	// 累计偏移量
	double sumCompRL = config[27], sumCompUD = config[28];
	// 累计运动距离
	// 历史偏移量
	double lastCompRL = config[29];
	printf("lastCompRL = %f\n", lastCompRL);

	// --- 输出结果初始化
	// 激活跟踪
	config[20] = 0;
	// 跟踪修正量(世界坐标xyz修正)
	config[21] = 0;
	config[22] = 0;
	config[23] = 0;

	// --- 偏移方向计算
	// 左右: 摆动方向, 右为正, zDir X tanDir
	double swingDir[3] = { 0,0,0 };
	vector_cross(zDir, tanDir, swingDir);
	if (vector_norm(swingDir)) {
		printf("swingDir error: %f, %f, %f\n", swingDir[0], swingDir[1], swingDir[2]);
		return 0;
	}
	// 上下: 深度方向, 下为正, tanDir X zDir X tanDir = tanDir X swingDir
	double depthDir[3] = { 0,0,0 };
	vector_cross(tanDir, swingDir, depthDir);
	if (vector_norm(depthDir)) {
		printf("depthDir error: %f, %f, %f\n", depthDir[0], depthDir[1], depthDir[2]);
		return 0;
	}
	printf("tanDir: %f, %f, %f; zDir: %f, %f, %f, swingDir: %f, %f, %f; depthDir: %f, %f, %f\n",
		tanDir[0], tanDir[1], tanDir[2],zDir[0], zDir[1], zDir[2],swingDir[0], swingDir[1], swingDir[2],depthDir[0], depthDir[1], depthDir[2]);

	// --- 偏移量计算
	// 左右基准修正
	if (fabs(tanDir[2]) < 0.2) {
		// 右侧向上，正向偏移(向下)左侧电流大
		if (swingDir[2] > 0.2)
			AR += offsetRL;
		// 左侧向上，正向偏移(向下)右侧电流大
		else if (swingDir[2] < -0.2)
			AL += offsetRL;
		// 平焊
	}
	// 上下基准修正
	AC += offsetUD;

	// 最大纠偏距离
	double maxShift = 0;
	if (masterDist > 0) {
		maxShift = fabs(masterDist * tan(6 * M_PI / 180));
	}
	printf("maxShift = %f, beg = %f, end = %f, dist = %f\n", maxShift, config[47], config[48], masterDist);

	// 左右跟踪
	double dArl = 0.0, compRL = 0.0;
	if (enable == 1 && enableRL == 1) {
		dArl = AR - AL;
		//compRL = fabs(dArl * gainRL);
		double smc_u = smc_process(smc[idx], dArl);
		compRL = lastCompRL + smc_u;
		printf("smc_u = %f, compRL = %f\n", dArl, compRL);
	}

	// 上下跟踪: dAud > 0 向上跟踪
	double dAud = 0.0, compUD = 0.0;
	if (enable == 1 && enableUD == 1 && ACref > 0) {
		dAud = AC - ACref;
		compUD = fabs(dAud * gainUD);
	}

	// 距离修正
	double sumDistSq = compRL * compRL + compUD * compUD;
	// 总修正大于最大纠偏，修正纠偏量
	printf("fabs(compRL) = %f\n", fabs(compRL));
	if (fabs(compRL) > maxShift) {
		//compRL = maxShift; 
		compRL = compRL > 0 ? maxShift : -maxShift;
	}
	if (compUD > maxShift) {
		compUD = maxShift;
	}

	if (enable == 1 && enableRL == 1) {
		//if (dArl > 0) {
		//	compRL *= -1.0;
		//	printf("<- %f\n", compRL);
		//}
		//else if (dArl < 0) {
		//	printf("-> %f\n", compRL);
		//}
		if (compRL < 0) {
			printf("<- %f\n", compRL);
		}
		else if (compRL > 0) {
			printf("-> %f\n", compRL);
		}

		// 左右累计偏移
		config[27] += compRL;
		config[29] = compRL;
		printf("dArl = %f. sumCompRL = %f\n", dArl, config[27]);

		// 偏移量
		config[20] = 1;
		config[21] += compRL * swingDir[0];
		config[22] += compRL * swingDir[1];
		config[23] += compRL * swingDir[2];
	}

	// 上下跟踪(左右跟踪幅度小时生效)
	if (enable == 1 && enableUD == 1 && ACref > 0 && fabs(dAud) > 5) {
		//printf("Aud = %f, AudRef = %f\n", AC, ACref);

		if (dAud > 0) {
			compUD *= -1;
			printf("Λ %f\n", compUD);
		}
		else if (dAud < 0) {
			printf("V %f\n", compUD);
		}

		// 上下累计偏移
		config[28] += compUD;
		printf("Aud = %f, AudRef = %f, sumCompUD = %f\n", AC, ACref, config[28]);

		config[20] = 1;
		config[21] += compUD * depthDir[0];
		config[22] += compUD * depthDir[1];
		config[23] += compUD * depthDir[2];
	}

	return 0;
}


/***********************************************************************
 *                        M A T H                                      *
 ***********************************************************************/
/**
 * 向量单位化
 * @param  [out] *vec  待初始化的向量
 * @return 0 - 正常返回; 1 - 模长为零
 */
int vector_norm(double *vec) {
	double norm = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
	if (norm < eps) {
		return 1;
	}

	vec[0] /= norm;
	vec[1] /= norm;
	vec[2] /= norm;

	return 0;
}

/**
 * 向量叉乘
 * @param  *va  向量a
 * @param  *vb  向量b
 * @param  [out] *ans  叉乘结果
 * @return 0 - 正常返回
 */
int vector_cross(double *va, double *vb, double *ans) {
	ans[0] = va[1] * vb[2] - va[2] * vb[1];
	ans[1] = va[2] * vb[0] - va[0] * vb[2];
	ans[2] = va[0] * vb[1] - va[1] * vb[0];
	return 0;
}

/**
 * 欧拉角转旋转矩阵
 * @param  *euler  欧拉角
 * @param  *seq    欧拉角顺序(0-x, 1-y, 2-z), e.g.{ 0,1,2 }
 * @param  [out] *mat    旋转矩阵
 * @return 0 - 正常返回
 *
 * 正运动欧拉角: (a,b,c) -> R = Rz(c)Ry(b)Rx(a)
 * FSAI 欧拉角:  (a,b,c) -> R = Rz(a)Ry(b)Rx(c)
 */
int euler2mat(double *euler, double *seq, double *mat) {
	double rx = euler[0], ry = euler[1], rz = euler[2];

	//double x0, x1, x2, y0, y1, y2, z0, z1, z2;
	mat[0] = cos(ry) * cos(rz);                          // x0
	mat[1] = sin(rx)*sin(ry)*cos(rz) - cos(rx)*sin(rz);  // y0
	mat[2] = sin(rz)*sin(rx) + cos(rz)*cos(rx)*sin(ry);  // z0
	mat[3] = cos(ry) * sin(rz);						     // x1
	mat[4] = sin(rx)*sin(ry)*sin(rz) + cos(rx)*cos(rz);  // y1
	mat[5] = cos(rx)*sin(rz)*sin(ry) - cos(rz)*sin(rx);  // z1
	mat[6] = -sin(ry);								     // x2
	mat[7] = sin(rx)*cos(ry);						     // y2
	mat[8] = cos(rx)*cos(ry);						     // z2

	return 0;
}

/**
 * 矩阵乘法
 * @param  *matA  左矩阵
 * @param  row    左矩阵行数
 * @param  col    左矩阵列数
 * @param  *matB  右矩阵
 * @param  colB   右矩阵列数
 * @param  [out] *ans    结果矩阵
 * @return 0 - 正常返回
 */
int matrix_multiply(double *matA, int row, int col, double *matB, int colB, double *ans) {
	// ans 的第 i 行
	for (int i = 0; i < row; ++i) {
		// ans 的第 j 列
		for (int j = 0; j < colB; ++j) {
			double tmp = 0.0;
			for (int k = 0; k < col; ++k) {
				// Cij = sum(Aik * Bkj, k=[0,col))
				tmp += matA[i*col + k] * matB[k*colB + j];
			}
			ans[i*colB + j] = tmp;
		}
	}

	//printf("ans = ");
	//for (int i = 0; i < col * colB; ++i) {
	//	printf("%f, ", ans[i]);
	//}
	//printf("\n");
	return 0;
}
