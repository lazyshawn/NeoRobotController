#include "MainViewModel.h"

MainViewModel::MainViewModel(QObject *parent) : QObject(parent),
    m_mainModel(new MainModel(this))
{
	// 数据更新定时器，降低刷新频率
	connect(&m_timer, &QTimer::timeout, this, &MainViewModel::on_timeout);
	m_timer.setInterval(200);
	m_timer.start();
}

int MainViewModel::cardConnectState() const {
	return m_mainModel->get_connectState();
}

double MainViewModel::speedRatio() const {
	return m_mainModel->get_speedRatio();
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

int MainViewModel::jog_move(int robotIdx, int axisIdx, int dir, int enable) {
	m_mainModel->jog_move(robotIdx, axisIdx, dir, enable);
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
	if (data.jPos.size() != modelData.jPos.size() || std::fabs(data.jPos[0] - modelData.jPos[0]) > 1e-3) {
		// 更新数据备份
		modelData.jPos = data.jPos;
		// 通知界面变更
		emit robot_pos_changed();
	}

	// --- 3. 下位机状态丢失或主动修改时进行同步

	// --- 4. 更新数据备份
	//modelData = data;
}
