#pragma once

#include <QThread>
#include <QtConcurrent/QtConcurrent>
#include <chrono>

#include "fsai_mainwindow.h"
#include "fsai_procedure.h"
#include "fsai_advanceConfigWindow.h"

#include "robot_interface/CoopRobot.h"

/**
* @brief  机器人状态，从下位机读取
*/
class RobotStatus {
public:
	bool online = false;
	int mode;
	std::vector<float> jPos, cPos;
};

/**
* @brief  当前应用状态，用于状态刷新
*/
class AppStatus {
public:
	// 当前选中机器人id
	int robotIdx = 0;
	
	// 主页面状态
	std::vector<MainWindowDisplayData> mainWindowData;
};


// 获取数据线程
class Worker : public QObject {
public:
	// 主窗口显示数据
	std::shared_ptr<MainWindowDisplayData> displayData;

	// 切换选中机器人
	void switch_robot(int idx);

Q_OBJECT
public slots:
	// 
	void doWork();

signals:
	// 数据同步
	void update_data(const MainWindowDisplayData data);

	void resultReady();
};



class FSAIApp : public QMainWindow {
Q_OBJECT

private:
	// 窗口初始化
	std::shared_ptr<MainWindow> mainWindow;

	// 数据
	std::shared_ptr<AppStatus> appStatus;
	std::shared_ptr<RobotStatus> robotStatus;

	// 获取数据线程
	QThread workerThread;
	Worker *worker = new Worker;

	// 
	std::shared_ptr<FSAIRobotInterface::Controller> ZController;
	std::shared_ptr<FSAIRobotInterface::RobotBase> robot;

	void connect_slot();

public:
	FSAIApp();
	~FSAIApp();

signals:
	void startWork(); // 触发Worker开始工作的信号

public slots:
	//void switch_online();
};

