#include <iostream>

#include "fsai_app.h"
#include <chrono>

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
