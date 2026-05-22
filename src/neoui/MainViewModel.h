#pragma once

#include "MainModel.h"

#include <QObject>
#include <QTimer>

#include "robot_interface/RobotTrajectory.h"

class MainViewModel : public QObject
{
    Q_OBJECT
	// 申明一个可绑定的属性，NOTIFY用于通知变更
	Q_PROPERTY(int autoMode READ autoMode NOTIFY autoMode_changed)
	Q_PROPERTY(int cardConnectState READ cardConnectState NOTIFY connectState_changed)
	Q_PROPERTY(double speedRatio READ speedRatio NOTIFY speedRatio_changed)

public:
    explicit MainViewModel(QObject *parent = nullptr);

	int cardConnectState() const;
	double speedRatio() const;
	int autoMode() const;

	// 提供一个命令供 View 调用
	Q_INVOKABLE void change_connectState();
	Q_INVOKABLE std::vector<double> get_jPos();
	Q_INVOKABLE void change_speedRatio(double ratio);
	Q_INVOKABLE void change_autoModel();

	// 点动接口
	Q_INVOKABLE int set_jogType(int robotIdx, int type);
	Q_INVOKABLE int jog_move(int robotIdx, int axisIdx, int dir, int enable);
	// 下发自动任务
	Q_INVOKABLE int set_auto_task(const FSAIRobotInterface::DiscreteTrajectory& trajectory);

private:
	void on_timeout();

public slots:

signals:
	void connectState_changed();
	void robot_pos_changed();
	void speedRatio_changed();
	void autoMode_changed();

private:
	// ViewModel 持有 Model
    MainModel *m_mainModel;
	//! 状态数据更新定时器，节流阀 + 数据缓冲
	QTimer m_timer;

	//! 状态数据备份
	ModelData modelData;
};
