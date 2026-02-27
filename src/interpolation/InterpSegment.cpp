
#include "interpolation/InterpSegment.h"


void ProcessInfo::reset() {
	lineNum = -1;
	processed = false;
}

/***********************************************************************
 *                        InterpSegment                                *
 ***********************************************************************/
int InterpSegment::set_data(const PointInfo& point, const MotionCfg& cfg, const MoveCmd& cmd) {
	pointInfo = point;
	motionCfg = cfg;
	moveCmd = cmd;

	return 0;
}



/***********************************************************************
 *                        B E Z I E R                                  *
 ***********************************************************************/
// 贝塞尔曲线位置
int bezier_positioin(int m, const double ctr[][3], double u, double ans[3]) {
	double *Q = (double *)malloc(sizeof(double) * (m + 1));

	// 第J维度
	for (int j = 0; j < 3; ++j) {
		for (int i = 0; i <= m; ++i) {
			Q[i] = ctr[i][j];
		}

		// DeCasteljau算法降维 m 次, 相比直接计算效率稍低，但是舍入误差更小
		for (int k = 1; k <= m; ++k) {
			for (int i = 0; i <= m - k; ++i) {
				Q[i] = (1.0 - u)*Q[i] + u * Q[i + 1];
			}
		}

		ans[j] = Q[0];
	}

	// 释放内存
	free(Q);
	return 0;
}

// 贝塞尔曲线导数
double bezier_derivatives(int m, const double ctr[][3], double u, double ans[3]) {
	// 贝塞尔曲线的导数是低阶的贝塞尔曲线
	double (*pnt)[3] = (double (*)[3])malloc(sizeof(double) * 3 * m);

	// 导数曲线控制点
	for (int i = 0; i < m; ++i) {
		for (int k = 0; k < 3; ++k) {
			pnt[i][k] = ctr[i + 1][k] - ctr[i][k];
		}
	}

	// b'(m, u) = m * b(m-1, u)
	bezier_positioin(m - 1, pnt, u, ans);
	for (int i = 0; i < 3; ++i) {
		ans[i] *= m;
	}

	// 释放内存
	free(pnt);
	return sqrt(ans[0]*ans[0] + ans[1]*ans[1] + ans[2]*ans[2]);
}

// 贝塞尔曲线段长度 - 辛普森积分
double bezier_dist(int m, const double ctr[][3], double a, double b, int n) {
	// 区间间隔，包含一个奇数节点和偶数节点
	double h = (b - a) / n;
	double deriv[3];

	// 奇数节点导数
	double so = bezier_derivatives(m, ctr, a + h * 0.5, deriv);
	// 偶数节点导数，除去0和2n
	double se = 0.0;
	// 闭区间等分成2n个小区间
	for (int i = 1; i < n; ++i) {
		so += bezier_derivatives(m, ctr, a + h * (i + 0.5), deriv);
		se += bezier_derivatives(m, ctr, a + h * i, deriv);
	}

	// ans = h/6 * (f(a) + 4*so + 2*se + f(b))
	double ans = bezier_derivatives(m, ctr, a, deriv) + bezier_derivatives(m, ctr, b, deriv);
	ans += 4 * so + 2 * se;
	ans *= h / 6;

	return ans;
}

// 贝塞尔曲线插补
double bezier_interp(int m, const double ctr[][3], double curU, double detS) {
	// 二分法
	double beg = curU, end = 1.0;
	int maxIteNum = 30;

	// 当前位置
	double curPos[3];
	bezier_positioin(m, ctr, curU, curPos);

	// 最大迭代次数
	double lastU = 1.0;
	for (int i = 0; i < maxIteNum; ++i) {
		// 中点参数
		double U = (beg + end) / 2;

		// 迭代位置
		double itePos[3];
		bezier_positioin(m, ctr, U, itePos);

		for (int k = 0; k < 3; ++k)
			itePos[k] -= curPos[k];
		double dis = sqrt(itePos[0]*itePos[0] + itePos[1] * itePos[1] + itePos[2] * itePos[2]);

		//if (fabs(dis - detS) < 1e-6)
		if (fabs(lastU - U) < 1e-2 * U && fabs(dis - detS) < 1e-6)
			return U;

		// 更新区间端点
		if (dis > detS)
			end = U;
		else
			beg = U;
	}

	return (beg + end) / 2;
}

