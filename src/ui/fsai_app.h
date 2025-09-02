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
//class RobotStatus {
//public:
//	bool online = false;
//	int mode;
//	std::vector<float> jPos, cPos;
//};

/**
* @brief  当前应用状态，用于状态刷新
*/
//class AppStatus {
//public:
//	// 当前选中机器人id
//	int robotIdx = 0;
//	
//	// 主页面状态
//	std::vector<MainWindowDisplayData> mainWindowData;
//};


// 获取数据线程
class Worker : public QObject {
	bool workerHealthy = true;

public:
	Worker();

	// 主窗口显示数据
	std::shared_ptr<MainWindowDisplayData> displayData;

	// 切换选中机器人
	void switch_robot(int idx);

Q_OBJECT
public slots:
	// 
	void doWork();
	void stopWork();

signals:
	// 数据同步
	void update_data(const MainWindowDisplayData& data);

	void resultReady();
};



class FSAIApp : public QMainWindow {
Q_OBJECT

private:
	// 窗口初始化
	std::shared_ptr<MainWindow> mainWindow;
	std::shared_ptr<AdvanceConfigWindow> advanceWindow;
	std::shared_ptr<ProcedureWindow> procedureWindow;

	// 数据
	//std::shared_ptr<AppStatus> appStatus;
	//std::shared_ptr<RobotStatus> robotStatus;

	// 获取数据线程
	QThread workerThread;
	Worker *worker;

	// 
	std::shared_ptr<FSAIRobotInterface::Controller> ZController;
	std::shared_ptr<FSAIRobotInterface::RobotBase> robot;

	void connect_slot();
	std::vector<float> read_list_from_tableWidget(const QTableWidget* table, int row, const std::vector<int>& idxList);

	// 显示工艺参数
	void display_procedure_data(int procIdx, int cmdIdx);
	void save_procedure_data();

	// 执行示教轨迹
	void append_teached_trajectory(int row, DiscreteTrajectory& traj);

	// 插入示教点参数设置按钮
	int insert_teach_point_config_button(const std::vector<int>& idxList);

	// 保存工程
	int save_project(std::string fileName);

public:
	FSAIApp();
	~FSAIApp();

signals:
	void startWork(); // 触发Worker开始工作的信号

public slots:
	//void switch_online();
};

