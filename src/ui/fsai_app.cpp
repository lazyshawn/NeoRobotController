
#include "fsai_app.h"

#include <random>
#include <chrono>
#include <iostream>

#include "robot_interface/ZMotionRobot.h"
#include "robot_interface/ZRVRobot.h"
#include "robot_interface/FSAIRobot.h"

FSAIRobotInterface::RobotGroupManager group;

void Worker::doWork() {

	// 随机种子
	std::default_random_engine engine(std::chrono::system_clock::now().time_since_epoch().count());
	std::normal_distribution<double> distribution(100, 10);

	// 加载默认参数
	displayData = std::shared_ptr<MainWindowDisplayData>(new MainWindowDisplayData);
	// 获取当前时间戳
	auto start = std::chrono::steady_clock::now();
	// 下次唤醒时间
	auto wakeUpTime = start;
	// 线程周期(ms)
	long long duration = 50;
	bool workerHealthy = true;

	while (workerHealthy) {
		// 获取当前主界面状态
		MainWindowDisplayData data;

		// 修改主界面状态
		FSAIRobotInterface::RobotStatus status;
		group.robotList[0]->get_rt_robot_status(status);
		data.jPos = status.jPos;
		data.cPos = status.cPos;

		// 暂停状态

		// 触发线程刷新
		emit update_data(data);

		// 设置下次唤醒时间
		wakeUpTime += std::chrono::milliseconds(duration);
		// 休眠
		auto now = std::chrono::steady_clock::now();

		if ((now - wakeUpTime).count() > 0) {
			//std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(wakeUpTime - start).count() << std::endl;
			long long detTime = std::chrono::duration_cast<std::chrono::milliseconds>(now - wakeUpTime).count();
			wakeUpTime += std::chrono::milliseconds((detTime / duration + 1) * duration);
		}
		else {
			std::this_thread::sleep_until(wakeUpTime);
		}
	}

	return;
}

void Worker::switch_robot(int idx) {
	displayData->selectedRobot = idx;
}


/**
* @brief  主窗口
*/
FSAIApp::FSAIApp() {
	// 资源初始化
	mainWindow = std::shared_ptr<MainWindow>(new MainWindow);

	// 状态数据初始化
	robotStatus = std::shared_ptr<RobotStatus>(new RobotStatus);

	// 显示主页
	mainWindow->show();

	ZController = std::shared_ptr<FSAIRobotInterface::Controller>(new FSAIRobotInterface::Controller);
	robot = std::shared_ptr<FSAIRobotInterface::RobotBase>(new FSAIRobotInterface::ZMotionRobot);
	//robot = std::shared_ptr<FSAIRobotInterface::RobotBase>(new FSAIRobotInterface::ZRVRobot);
	//robot = std::shared_ptr<FSAIRobotInterface::RobotBase>(new FSAIRobotInterface::FSAIRobot);

	ZController->lazy_connect();
	robot->set_ZController(ZController);
	group.new_robot(robot);
	group.start_thread();

	// 连接信号和槽
	connect_slot();

	// 初始状态
	mainWindow->ui->radioButton->setChecked(true);

	// 开启状态刷新线程
	worker->moveToThread(&workerThread);
	QObject::connect(&workerThread, &QThread::started, worker, &Worker::doWork);
	workerThread.start();

}

FSAIApp::~FSAIApp() {
}



void FSAIApp::connect_slot() {

	/* ********************** 监控页面 ********************** */
	// 状态刷新线程触发显示更新
	connect(worker, &Worker::update_data, mainWindow.get(), (void (MainWindow::*)(MainWindowDisplayData)) &MainWindow::update_display);


	/* ********************** 点动页面 ********************** */
	//	切换点动模式
	QObject::connect(mainWindow->jogTypeBtn[0], &QRadioButton::clicked, this, [&]() {
		for (size_t i = 0; i < 6; ++i) {
			mainWindow->jogMoveLabel[i]->setText("J" + QString::number(i));
		}
	});
	for (size_t i = 1; i < mainWindow->jogTypeBtn.size(); ++i) {
		QObject::connect(mainWindow->jogTypeBtn[i], &QRadioButton::clicked, this, [&]() {
			mainWindow->jogMoveLabel[0]->setText("X");
			mainWindow->jogMoveLabel[1]->setText("Y");
			mainWindow->jogMoveLabel[2]->setText("Z");
			mainWindow->jogMoveLabel[3]->setText("A");
			mainWindow->jogMoveLabel[4]->setText("B");
			mainWindow->jogMoveLabel[5]->setText("C");
		});
	}
	// 点动
	for (int i = 0; i < 9; ++i) {
		QObject::connect(mainWindow->jogMoveBtn[2 * i], &QPushButton::pressed, this, [&, i]() {
			// 点动模式
			int type = 0;
			for (size_t j = 0; j < mainWindow->jogTypeBtn.size(); ++j) {
				if (mainWindow->jogTypeBtn[j]->isChecked()) {
					type = j;
					break;
				}
			}
			group.robotList[0]->jog_moving(type, i, -1, 1);
		});
		QObject::connect(mainWindow->jogMoveBtn[2 * i], &QPushButton::released, this, [&, i]() {
			int type = 0;
			for (size_t j = 0; j < mainWindow->jogTypeBtn.size(); ++j) {
				if (mainWindow->jogTypeBtn[j]->isChecked()) {
					type = j;
					break;
				}
			}
			group.robotList[0]->jog_moving(type, i, 0, 0);
		});

		QObject::connect(mainWindow->jogMoveBtn[2 * i + 1], &QPushButton::pressed, this, [&, i]() {
			int type = 0;
			for (size_t j = 0; j < mainWindow->jogTypeBtn.size(); ++j) {
				if (mainWindow->jogTypeBtn[j]->isChecked()) {
					type = j;
					break;
				}
			}
			group.robotList[0]->jog_moving(type, i, 1, 1);
		});
		QObject::connect(mainWindow->jogMoveBtn[2 * i + 1], &QPushButton::released, this, [&, i]() {
			int type = 0;
			for (size_t j = 0; j < mainWindow->jogTypeBtn.size(); ++j) {
				if (mainWindow->jogTypeBtn[j]->isChecked()) {
					type = j;
					break;
				}
			}
			group.robotList[0]->jog_moving(type, i, 0, 0);
		});
	}


	/* ********************** 控制页面 ********************** */
	// 切换手自动模式
	connect(mainWindow->ui->checkBox_2, &QCheckBox::toggled, this, [&](bool checked) {
		group.robotList[worker->displayData->selectedRobot]->switch_auto(checked);
	});

	// 切换选中机器人
	for (size_t i = 0; i < 4; ++i) {
		QObject::connect(mainWindow->robotButton[i], &QPushButton::pressed, this, [&, i]() {
			worker->switch_robot(i);
		});
	}

	/* ********************** 示教页面 ********************** */
	// 开始
	connect(mainWindow->ui->pushButton_4, &QPushButton::pressed, this, [&]() {
	});
	// 执行当前
	connect(mainWindow->ui->pushButton_7, &QPushButton::pressed, this, [&]() {
		// 当前行转换为轨迹
		DiscreteTrajectory traj;

		// 当前选中示教点
		int row = mainWindow->ui->tableWidget->currentRow();

		// 运动类型
	});
	// 暂停
	connect(mainWindow->ui->pushButton_5, &QPushButton::pressed, this, [&]() {
		group.robotList[worker->displayData->selectedRobot]->task_pause();
		mainWindow->ui->pushButton_5->setText("Resume");
		//group.robotList[mainWindow->displayData->selectedRobot]->task_resume();
		//mainWindow->ui->pushButton_5->setText("Pause");
	});
	// 停止
	connect(mainWindow->ui->pushButton_6, &QPushButton::pressed, this, [&]() {
		group.robotList[worker->displayData->selectedRobot]->task_stop();
	});



	/* ********************** 日志页面 ********************** */
	// 按键下发指令
	QObject::connect(mainWindow->ui->pushButton_8, &QPushButton::pressed, this, [&]() {
		auto cmd = mainWindow->ui->lineEdit->text();

		// 下发指令，获取返回值
		std::string ack;
		int ret = group.robotList[0]->send_command(cmd.toStdString(), ack);

		if (ret)
			mainWindow->ui->textBrowser->append("error return: " + QString::number(ret));
		else
			mainWindow->ui->textBrowser->append(QString::fromStdString(ack));
	});
	// 回车下发指令
	QObject::connect(mainWindow->ui->lineEdit, &QLineEdit::returnPressed, this, [&]() {
		auto cmd = mainWindow->ui->lineEdit->text();

		// 下发指令，获取返回值
		std::string ack;
		int ret = group.robotList[0]->send_command(cmd.toStdString(), ack);
		if (ret)
			mainWindow->ui->textBrowser->append("error return: " + QString::number(ret));
		else
			mainWindow->ui->textBrowser->append(QString::fromStdString(ack));
	});

}
