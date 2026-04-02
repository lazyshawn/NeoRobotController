
#ifdef _MSC_VER
#include "data_process/track_utility.h"
#else
#include "track_utility.h"
#endif


// 误差临界值
static const double eps = 1e-6;

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
int matrix_multiply_in_vector(double *matA, int row, int col, double *matB, int colB, double *ans) {
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

	return 0;
}

// 矩阵初始化
MatrixXd *matrix_new(int rows, int cols, double val) {
	MatrixXd *q = (MatrixXd *)malloc(sizeof(MatrixXd));
	if (!q)
		return NULL;

	q->rows = rows;
	q->cols = cols;
	if (!(q->data = (double *)malloc(sizeof(double) * q->rows * q->cols))) {
		free(q);
		return NULL;
	}

	for (int i = 0; i < q->rows; ++i) {
		for (int j = 0; j < q->cols; ++j) {
			q->data[i*q->cols + j] = val;
		}
	}

	return q;
}

// 单位矩阵初始化
MatrixXd *matrix_new_identity(int rows) {
	MatrixXd *q = (MatrixXd *)malloc(sizeof(MatrixXd));
	if (!q)
		return NULL;

	q->rows = q->cols = rows;
	if (!(q->data = (double *)malloc(sizeof(double) * q->rows * q->cols))) {
		free(q);
		return NULL;
	}

	for (int i = 0; i < q->rows; ++i) {
		for (int j = 0; j < q->cols; ++j) {
			q->data[i*q->cols + j] = (i == j) ? 1.0 : 0.0;
		}
	}

	return q;
}

// 矩阵析构
void matrix_delete(MatrixXd * q) {
	free(q->data);

	free(q);
}

// 矩阵格式化输出
int matrix_cout(const MatrixXd * q) {
	for (int i = 0; i < q->rows; ++i) {
		for (int j = 0; j < q->cols; ++j) {
			printf("%f, ", q->data[i*q->cols + j]);
		}
		printf("\n");
	}
	printf("\n");
	return 0;
}

// 矩阵拷贝
MatrixXd * matrix_new_copy(const MatrixXd * q) {
	MatrixXd *ans = (MatrixXd *)malloc(sizeof(MatrixXd));

	ans->rows = q->rows;
	ans->cols = q->cols;
	ans->data = (double *)malloc(sizeof(double) * ans->rows * ans->cols);
	for (int i = 0; i < ans->rows*ans->cols; ++i) {
		ans->data[i] = q->data[i];
	}

	return ans;
}

// 获取元素
int matrix_get(const MatrixXd *mat, int row, int col, double *ans) {
	// 索引越界
	if (mat->rows - row < 0 || mat->cols - col < 0)
		return 1;

	*ans = mat->data[row*mat->cols + col];

	return 0;
}

// 设置元素
int matrix_set(MatrixXd *mat, int row, int col, double val) {
	// 索引越界
	if (mat->rows - row < 0 || mat->cols - col < 0)
		return 1;

	mat->data[row*mat->cols + col] = val;

	return 0;
}

// 获取矩阵块
int matrix_get_block(const MatrixXd *matA, int row, int col, MatrixXd *matB) {
	// 输入合法性检测
	if (row + matB->rows - matA->rows < 0 || col + matB->cols - matA->cols < 0)
		return 1;

	for (int i = 0; i < matB->rows; ++i) {
		for (int k = 0; k < matB->cols; ++k) {
			double tmp = 0.0;
			matrix_get(matA, row + i, col + k, &tmp);
			matrix_set(matB, i, k, tmp);
		}
	}
	return 0;
}

// 设置矩阵块
int matrix_set_block(MatrixXd *matA, int row, int col, const MatrixXd *matB) {
	// 输入合法性检测
	if (row + matB->rows - matA->rows < 0 || col + matB->cols - matA->cols < 0)
		return 1;

	for (int i = 0; i < matB->rows; ++i) {
		for (int k = 0; k < matB->cols; ++k) {
			double tmp = 0.0;
			matrix_get(matB, i, k, &tmp);
			matrix_set(matA, row + i, col + k, tmp);
		}
	}
	return 0;
}

// 矩阵乘法
int matrix_multiply(const MatrixXd *matA, const MatrixXd *matB, MatrixXd *ans) {
	// 行列数校验
	if (matA->cols - matB->rows != 0)
		return 1;

	if ((ans->rows - matA->rows != 0) || (ans->cols - matB->cols != 0)) {
		ans->rows = matA->rows;
		ans->cols = matB->cols;
		if (ans->data) {
			free(ans->data);
		}
		ans->data = (double *)malloc(sizeof(double) * ans->rows * ans->cols);
	}
	matrix_multiply_in_vector(matA->data, matA->rows, matA->cols, matB->data, matB->cols, ans->data);

	return 0;
}


// ============== LUP分解辅助函数 ==============
/**
 * @brief  LUP分解函数
 * @param  A    [out] 输入nXn矩阵A，输出交换主元后的L和U组合矩阵（L的对角线为1）
 * @param  P    [out] 置换向量，P[i]表示第i行被交换到哪一行
 * @param  sign 行列式的符号（用于计算行列式）
 * @return
 *   - 0: 表示成功
 *   - 1: 表示奇异矩阵，主元为0
 */
int LUP_decompose(MatrixXd* A, MatrixXd* P, int* sign) {
	int n = A->rows;

	// 初始化置换向量和行列式符号
	for (int i = 0; i < n; i++)
		P->data[i] = 1.0 * i;
	*sign = 1;

	for (int i = 0; i < n; i++) {
		// 部分主元选择：寻找第i列中绝对值最大的元素
		double max_val = 0.0;
		int max_row = i;
		for (int k = i; k < n; k++) {
			double abs_val;
			matrix_get(A, k, i, &abs_val);
			abs_val = fabs(abs_val);

			if (abs_val > max_val) {
				max_val = abs_val;
				max_row = k;
			}
		}

		// 检查矩阵是否奇异
		if (max_val < eps) {
			printf("警告: 矩阵在行 %d 处奇异或接近奇异\n", i);
			return 1;
		}

		// 如果需要，交换行
		if (max_row != i) {
			// 交换置换向量中的索引
			int temp = P->data[i];
			P->data[i] = P->data[max_row];
			P->data[max_row] = temp;

			// 交换矩阵的行
			MatrixXd *rowCur = matrix_new(1, n, 0), *rowPivot = matrix_new(1, n, 0);

			matrix_get_block(A, i, 0, rowCur);
			matrix_get_block(A, max_row, 0, rowPivot);
			matrix_set_block(A, i, 0, rowPivot);
			matrix_set_block(A, max_row, 0, rowCur);

			matrix_delete(rowCur);
			matrix_delete(rowPivot);

			// 更新行列式符号，交换一次更新一次
			*sign = -(*sign);
		}

		// 高斯消元
		double pivot = 0.0;
		matrix_get(A, i, i, &pivot);
		for (int k = i + 1; k < n; k++) {
			double factor = 0.0;
			matrix_get(A, k, i, &factor);
			factor /= pivot;
			// 存储乘子到L部分
			matrix_set(A, k, i, factor);

			// 计算消元后的U部分
			for (int j = i + 1; j < n; j++) {
				double valP = 0.0, valU = 0.0;
				matrix_get(A, i, j, &valP);  // 主元行第j列
				matrix_get(A, k, j, &valU);  // 当前行第j列
				matrix_set(A, k, j, valU - factor * valP);
			}
		}
	}

	return 0;
}

// 前向替代（解 Ly = Pb）
void forward_substitution(const MatrixXd* LU, const MatrixXd* Pb, MatrixXd* y) {
	int n = LU->rows;
	for (int i = 0; i < n; i++) {
		double sum = 0.0;
		for (int j = 0; j < i; j++) {
			double tmp = 0.0;
			matrix_get(LU, i, j, &tmp);
			sum += tmp * y->data[j];
		}
		y->data[i] = Pb->data[i] - sum;  // L的对角线为1
	}
}

// 后向替代（解 Ux = y）
void backward_substitution(const MatrixXd* LU, const MatrixXd* y, MatrixXd* x) {
	int n = LU->rows;
	for (int i = n - 1; i >= 0; i--) {
		double sum = 0.0;
		for (int j = i + 1; j < n; j++) {
			double tmp = 0.0;
			matrix_get(LU, i, j, &tmp);
			sum += tmp * x->data[j];
		}
		double pivot = 0.0;
		matrix_get(LU, i, i, &pivot);
		x->data[i] = (y->data[i] - sum) / pivot;
	}
}

// ============== LUP求逆主函数 ==============
/**
 * @brief  使用LUP分解求矩阵的逆
 * @param  A      原始矩阵
 * @param  A_inv  [out] 逆矩阵
 * @return
 *   - 0: 表示成功
 *   - 1: 表示不是方阵
 *   - 2: 表示LUP分解失败
 */
int matrix_LUP_inverse(const MatrixXd* A, MatrixXd* A_inv) {
	if (A->rows - A->cols != 0) {
		printf("错误: 矩阵不是方阵\n");
		return -1;
	}

	int n = A->rows;

	// 步骤1: 复制矩阵并执行LUP分解
	MatrixXd *LU = matrix_new_copy(A);
	MatrixXd *P = matrix_new(n, 1, 0);
	int sign;

	if (LUP_decompose(LU, P, &sign) != 0) {
		matrix_delete(LU);
		matrix_delete(P);
		return 2;
	}

	// 步骤2: 对单位矩阵的每一列求解
	for (int col = 0; col < n; col++) {
		// 创建单位矩阵的第col列（经过置换）
		MatrixXd *Pb = matrix_new(n, 1, 0);
		Pb->data[(int)(P->data[col])] = 1.0;  // 注意：这里根据P进行置换

		// 解 L*y = Pb
		MatrixXd *y = matrix_new(n, 1, 0);
		forward_substitution(LU, Pb, y);

		// 解 U*x = y
		MatrixXd *x = matrix_new(n, 1, 0);
		backward_substitution(LU, y, x);

		// 将解存储到逆矩阵的相应列
		for (int i = 0; i < n; i++) {
			matrix_set(A_inv, i, col, x->data[i]);
		}

		matrix_delete(y);
		matrix_delete(x);
		matrix_delete(Pb);
	}

	// 清理内存
	matrix_delete(LU);
	matrix_delete(P);

	return 0;
}
