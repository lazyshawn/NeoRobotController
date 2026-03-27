
#include "AuxMatrix.h"

static const double dim_EPS = 1e-6;

/***********************************************************************
 *                        A U X I L I A R Y -- M A T R I X             *
 *                        矩阵运算辅助函数                             *
 ***********************************************************************/
// ============== 基础矩阵运算辅助函数 ==============

// 向量单位化
int vector_norm(double *vec) {
	return 0;
}

// 向量叉乘
int vector_cross(double *va, double *vb, double *ans) {
	return 0;
}

// 欧拉角转旋转矩阵
int euler2mat(double *euler, double *seq, double *mat) {
	return 0;
}

// 矩阵乘法
int matrix_multiply_in_vector(double *matA, int row, int col, double *matB, int colB, double *ans) {
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
		if (max_val < dim_EPS) {
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


/***********************************************************************
 *                        M A T R I X                                  *
 ***********************************************************************/
// --- 接口函数

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

MatrixXd *matrix_from_array(int rows, int cols, const double *val, int num) {
	MatrixXd *q = (MatrixXd *)malloc(sizeof(MatrixXd));
	if (!q)
		return NULL;

	q->rows = rows;
	q->cols = cols;
	if (!(q->data = (double *)malloc(sizeof(double) * q->rows * q->cols))) {
		free(q);
		return NULL;
	}

	// 实际使用元素个数
	int replace = rows * cols;
	if (num > 0 && num < replace)
		replace = num;

	int idx = 0;
	for (int i = 0; i < q->rows; ++i) {
		for (int j = 0; j < q->cols; ++j) {
			// 剩余元素用0填充
			q->data[i*q->cols + j] = (idx < replace) ? val[idx] : 0.0;
			idx++;
		}
	}

	return q;
}

// 矩阵析构
void matrix_delete(MatrixXd * q) {
	free(q->data);

	free(q);
}

// 矩阵拷贝
MatrixXd * matrix_copy(const MatrixXd * q) {
	MatrixXd *ans = (MatrixXd *)malloc(sizeof(MatrixXd));

	ans->rows = q->rows;
	ans->cols = q->cols;
	ans->data = (double *)malloc(sizeof(double) * ans->rows * ans->cols);
	for (int i = 0; i < ans->rows*ans->cols; ++i) {
		ans->data[i] = q->data[i];
	}

	return ans;
}

// 矩阵格式化输出
int matrix_cout(const MatrixXd * q) {
	for (int i = 0; i < q->rows; ++i) {
		for (int j = 0; j < q->cols; ++j) {
			printf("%f, ", q->data[i*q->cols + j]);
		}
		printf("\n");
	}

	return 0;
}

int matrix_to_array(const MatrixXd * q, double *val, int num) {
	int idx = 0;
	for (int i = 0; i < q->rows; ++i) {
		for (int j = 0; j < q->cols; ++j) {
			if (num > 0 && idx < num) {
				val[idx] = q->data[i*q->cols + j];
			}
			idx++;
		}
	}

	// 不足的用0补充
	for (int i = q->rows*q->cols; i < num; ++i) {
		val[i] = 0.0;
	}
	return 0;
}


// 获取元素
int matrix_get(const MatrixXd *mat, int row, int col, double *ans) {
	// 索引越界
	if (mat->rows - row < 0 || mat->cols - col < 0)
		return 1;

	*ans = mat->data[row*mat->cols + col];

	return 0;
}
double matrix_at(const MatrixXd *mat, int row, int col) {
	// 索引越界
	if (mat->rows - row < 0 || mat->cols - col < 0)
		return 0.0;

	return mat->data[row*mat->cols + col];
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


// --- 矩阵校验
// 矩阵大小校验
int matrix_same_size(MatrixXd *mat1, MatrixXd *mat2) {
	return (mat1->cols - mat2->cols == 0 && mat1->rows - mat2->rows == 0);
}

// --- 基础矩阵运算
// 矩阵乘常量
int matrix_scale(MatrixXd* q, double scale) {
	for (int i = 0; i < q->rows; ++i) {
		for (int j = 0; j < q->cols; ++j) {
			q->data[i*q->cols + j] *= scale;
		}
	}

	return 0;
}

// 矩阵加减法
int matrix_plus(double k1, MatrixXd *mat1, double k2, MatrixXd *mat2, MatrixXd *ans) {
	// 矩阵大小校验
	if (!matrix_same_size(ans, mat1) || !matrix_same_size(ans, mat2))
		return 1;

	for (int i = 0; i < ans->rows; ++i) {
		for (int j = 0; j < ans->cols; ++j) {
			ans->data[i*ans->cols + j] = k1 * mat1->data[i*mat1->cols + j] + k2 * mat2->data[i*mat2->cols + j];
		}
	}

	return 0;
}

// 矩阵减法
int matrix_minus(MatrixXd *mat1, MatrixXd *mat2, MatrixXd *ans) {
	// 矩阵大小校验
	if (!matrix_same_size(ans, mat1) || !matrix_same_size(ans, mat2))
		return 1;

	for (int i = 0; i < ans->rows; ++i) {
		for (int j = 0; j < ans->cols; ++j) {
			ans->data[i*ans->cols + j] = mat1->data[i*mat1->cols + j] - mat2->data[i*mat2->cols + j];
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

// 矩阵范数
double matrix_norm(const MatrixXd *mat) {
	int num = mat->cols * mat->rows;

	double ans = 0.0;
	for (int i = 0; i < num; ++i) {
		ans += mat->data[i] * mat->data[i];
	}
	ans = sqrt(ans);

	return ans;
}

// 矩阵单位化
int matrix_normalize(MatrixXd *mat) {
	double norm = matrix_norm(mat);

	for (int i = 0; i < mat->rows; ++i) {
		for (int j = 0; j < mat->cols; ++j) {
			mat->data[i*mat->cols + j] /= norm;
		}
	}

	return 0;
}

// 矩阵内积
double matrix_inner_product(const MatrixXd *matA, const MatrixXd *matB) {
	// 行列数校验
	if (matA->cols - matB->cols != 0 || matA->rows - matB->rows != 0)
		return 0.0;

	double ans = 0.0;
	for (int i = 0; i < matA->rows; ++i) {
		for (int j = 0; j < matA->cols; ++j) {
			ans += matA->data[i*matA->cols + j] * matB->data[i*matB->cols + j];
		}
	}

	return ans;
}

// 矩阵外积
int matrix_outer_product(const MatrixXd *matA, const MatrixXd *matB, MatrixXd *ans) {
	if (matA->rows - 3 != 0 || matA->cols - 1 != 0)
		return 1;
	if (matA->cols - matB->cols != 0 || matA->rows - matB->rows != 0)
		return 2;

	ans->data[0] = matA->data[1] * matB->data[2] - matA->data[2] * matB->data[1];
	ans->data[1] = -(matA->data[0] * matB->data[2] - matA->data[2] * matB->data[0]);
	ans->data[2] = matA->data[0] * matB->data[1] - matA->data[1] * matB->data[0];

	return 0;
}

/**
 * @brief  LUP求逆主函数
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
	MatrixXd *LU = matrix_copy(A);
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
