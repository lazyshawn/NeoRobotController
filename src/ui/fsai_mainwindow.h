#pragma once

#include <thread>
#include <memory>
#include <mutex>
#include <condition_variable>

#include <QDesktopWidget>
#include <QToolTip>
#include "ui_mainWindow.h"

#include "fsai_display_data.h"


// 主窗口显示数据
//struct MainWindowDisplayData {
//	// 机器人数量
//	int robotNum = 4;
//	// 当前选中机器人
//	int selectedRobot = 0;
//
//	// 全局配置
//	// 算法类型
//
//	// 运行状态: <离线, 空闲, 运行, 警告, 异常>
//	std::vector<int> runStatus = { 0,0,0,0 };
//	// 机器人状态
//	std::vector<int> errorStatus = { 0,0,0,0 };
//
//	// 机器人运行状态 <关节/世界/机器人/工具> <手动/自动>
//	std::vector<int> robotMode = { 1,1,1,1 };
//
//	// 机器人位置
//	std::vector<float> jPos;
//	std::vector<float> cPos;
//
//	// IO状态
//
//	// 示教轨迹<序号, 运动类型, 工艺号, 关节位置, TCP位置, 地轨位置>
//	std::vector<int> trajectory;
//};
//Q_DECLARE_METATYPE(MainWindowDisplayData);

// 主窗口观察者接口
class MainWindowSubscriber {
public:
	virtual ~MainWindowSubscriber() = default;
	virtual void update_MainWindow(const MainWindowDisplayData& data) = 0;
};

// 主窗口发布者接口
class MainWindowPublisher {
public:
	virtual ~MainWindowPublisher() = default;
	virtual void attach_MainWindow(MainWindowSubscriber* subscriber) = 0;
	//virtual void detach_MainWindow(MainWindowSubscriber* subscriber) = 0;
	virtual void notify_MainWindow() = 0;
};



/**
* @brief  主窗口
*/
class MainWindow : public QMainWindow {
public:
	// 主页面
	std::shared_ptr<Ui_MainWindow> ui;
	// 主窗口显示数据
	//std::shared_ptr<MainWindowDisplayData> displayData;

	// 控制卡类


public:
	MainWindow();
	~MainWindow();

	// 机器人标签
	std::vector<QPushButton*> robotButton;

	// 点动标签
	std::vector<QPushButton *> jogMoveBtn;
	std::vector<QLabel *> jogMoveLabel;
	std::vector<QRadioButton *> jogTypeBtn;

	// 当前记录的轨迹运动参数
	std::vector<std::map<int, std::vector<float>>> moveCfg;

private:

	void set_up_ui();
	void connect_slot();
	void record_teach_point();
	void delete_teach_point();

Q_OBJECT
public slots:
	void update_display();
	void update_display(const MainWindowDisplayData& data);

};

