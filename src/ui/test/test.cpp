#include <iostream>

#include "fsai_app.h"
#include <chrono>

int lup_decomposition(int n, double A[4][4], double L[4][4], double U[4][4], double P[4][4]) {
	double eps = 1e-6;

	// 矩阵初始化
	U = A;
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < n; ++j) {
			L[i][j] = i==j ? 1.0 : 0.0;
			P[i][j] = i == j ? 1.0 : 0.0;
		}
	}

	// 
	for (int i = 0; i < n; ++i) {
		// 选择主元
		int pivot = i;
		for (int j = i + 1; j < n; ++j) {
			if (fabs(U[j][i]) > fabs(U[pivot][i])) {
				pivot = j;
			}
		}

		// 交换行: pivot <-> i
		if (pivot != i) {
			// U 只用交换 [i,n) 列
			for (int j = i; j < n; ++j) {

			}
		}

		// 矩阵奇异
		if (U[i][i] < eps) {

		}

		// 消元
		for (int j = i + 1; j < n; ++j) {

		}
	}
	return 0;
}

int main(int argc, char** argv) {

	// 4x4 矩阵
	double a[4][4], b[4][4];


	QApplication app(argc, argv);

	// Ref: [注册自定义类型](https://www.cnblogs.com/luoxiang/p/17888430.html)
	qRegisterMetaType<MainWindowDisplayData>("MainWindowDisplayData&");
	//qRegisterMetaType<std::map<int, std::vector<float>>>("std::map<int, std::vector<float>>&");

	FSAIApp fsaiApp;

	return app.exec();
}
