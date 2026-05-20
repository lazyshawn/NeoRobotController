#pragma once

#include "ui_mainWindow.h"
#include "MainViewModel.h"

#include <QMainWindow>

class MainWindow : public QMainWindow {
	Q_OBJECT
public:
	explicit MainWindow(QWidget *parent = nullptr);
	~MainWindow();

protected:
	void showEvent(QShowEvent* event) override;

private slots:
	void on_connectState_changed();
	void on_robot_pos_changed();
	void on_speedRatio_changed();
	void on_autoMode_changed();

private:
	Ui::MainWindow *ui;
	MainViewModel *m_viewModel;

	// 修改为从 Model 获取
	int m_robotIdx = 0;

	// 界面初始化
	void setup_ui();
	// 数据绑定
	void bind_viewmodel();

	void robot_change(int idx);
	void set_auto_task();
};
