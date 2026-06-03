#include "MainViewModel.h"
#include <cmath>

MainViewModel::MainViewModel(QObject *parent) : QObject(parent),
    m_mainModel(new MainModel(this))
{
	// 数据更新定时器，降低刷新频率
	connect(&m_timer, &QTimer::timeout, this, &MainViewModel::on_timeout);
	m_timer.setInterval(100);
	m_timer.start();
}

int MainViewModel::cardConnectState() const {
	return m_mainModel->get_connectState();
}

double MainViewModel::speedRatio() const {
	return m_mainModel->get_speedRatio();
}

int MainViewModel::autoMode() const {
	return m_mainModel->get_autoMode();
}

void MainViewModel::change_connectState() {
	int state = m_mainModel->get_connectState();
	if (state > 0) {
		m_mainModel->disconnect();
	}
	else {
		// 读取写入的控制卡地址
		std::string addr = "";
		// 连接控制卡
		m_mainModel->connect("");
	}
	// 修改完后通知 View 进行变更
	emit connectState_changed();
}

void MainViewModel::change_speedRatio(double ratio) {
	m_mainModel->set_speedRatio(ratio);
	emit speedRatio_changed();
}

void MainViewModel::change_autoModel() {
	int state = m_mainModel->get_autoMode();
	m_mainModel->set_autoMode(state <= 0);
}

int MainViewModel::set_jogType(int robotIdx, int type) {
	m_mainModel->set_jogType(robotIdx, type);
	return 0;
}

int MainViewModel::jog_move(int robotIdx, int axisIdx, int dir, int enable) {
	m_mainModel->jog_move(robotIdx, axisIdx, dir, enable);
	return 0;
}

int MainViewModel::set_auto_task(const FSAIRobotInterface::DiscreteTrajectory& trajectory) {
	m_mainModel->push_trajectory(trajectory);
	return 0;
}

std::vector<double> MainViewModel::get_jPos() {
	return modelData.jPos;
}

void MainViewModel::on_timeout() {
	// --- 1. 获取新数据
	ModelData data;
	m_mainModel->get_modelData(data);

	// --- 2. 脏标记检测
	// 2.1 机器人位置
	double sum = 0.0;
	if (data.jPos.size() != 9 || modelData.jPos.size() != 9) {
		sum = 1e3;
	}
	else {
		for (int i = 0; i < 9; ++i) {
			sum += std::fabs(data.jPos[i] - modelData.jPos[i]);
		}
	}
	if (sum > 1e-6) {
		// 更新数据备份
		modelData.jPos = data.jPos;
		// 通知界面变更
		emit robot_pos_changed();
	}
	// 2.2 手自动模式
	if (modelData.autoMode != data.autoMode) {
		modelData.autoMode = data.autoMode;
		emit autoMode_changed();
	}

	// --- 3. 下位机状态丢失或主动修改时进行同步

	// --- 4. 更新数据备份
	//modelData = data;
}
