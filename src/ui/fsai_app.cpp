
#include "fsai_app.h"

#include <random>
#include <chrono>
#include <iostream>

#include <QFileDialog> // 打开文件浏览器选择文件
#include <QTextCodec>  // 中文路径
#include <QtNetwork/QHostAddress>

#include "robot_interface/ZMotionRobot.h"
#include "robot_interface/ZRVRobot.h"
#include "robot_interface/FSAIRobot.h"

#include "nlohmann/json.hpp"

FSAIRobotInterface::RobotGroupManager group;

// --- 辅助函数
// Ref: https ://leetcode.cn/problems/validate-ip-address/solutions/1521467/yan-zheng-ipdi-zhi-by-leetcode-solution-kge5/
int validIPAddress(std::string queryIP);


Worker::Worker() {
	// 加载默认参数
	// 主窗口显示数据
	displayData = std::shared_ptr<MainWindowDisplayData>(new MainWindowDisplayData);

	// 最大机器人个数
	displayData->robotNum = 1;
	// 获取系统当前的时间: yyMMdd_hhmmss
	QDateTime dateTime = QDateTime::currentDateTime();
	displayData->projectName = "project/" + dateTime.toString("yyMMdd").toStdString() + ".json";
	// 每个机器人保存一份轨迹
	displayData->teachPoints = std::vector<std::string>(displayData->robotNum);
	// 算法类型
	displayData->interpAlgo = 0;
}

void Worker::doWork() {

	// 随机种子
	std::default_random_engine engine(std::chrono::system_clock::now().time_since_epoch().count());
	std::normal_distribution<double> distribution(100, 10);

	// 获取当前时间戳
	auto start = std::chrono::steady_clock::now();
	// 下次唤醒时间
	auto wakeUpTime = start;
	// 线程周期(ms)
	long long duration = 50;

	while (workerHealthy) {
		
		int robotIdx = displayData->selectedRobot;
		// --- 修改主界面状态
		FSAIRobotInterface::RobotStatus status;
		group.robotList[robotIdx]->get_rt_robot_status(status);

		// 机器人当前位置
		displayData->jPos = status.jPos;
		displayData->cPos = status.cPos;

		// 手自动模式
		FSAIRobotInterface::set_bit(displayData->robotMode[robotIdx], 0, status.autoMode > 0);

		for (size_t i = 0; i < displayData->robotNum; ++i) {
			group.robotList[i]->get_rt_robot_status(status);

			// 运行状态复位
			displayData->runStatus[i] = 0;
			// 在线/离线
			if (!FSAIRobotInterface::get_bit(status.lowerStatus, 6)) {
				FSAIRobotInterface::set_bit(displayData->runStatus[i], 0, true);
			}
			// 空闲 / 运行中
			if (FSAIRobotInterface::get_bit(status.lowerStatus, 0)) {
				FSAIRobotInterface::set_bit(displayData->runStatus[i], 2, true);
			}
			else {
				FSAIRobotInterface::set_bit(displayData->runStatus[i], 1, true);
			}
			// 暂停 / 警告
			if (FSAIRobotInterface::get_bit(status.lowerStatus, 1)) {
				FSAIRobotInterface::set_bit(displayData->runStatus[i], 3, true);
			}
			// 下位机异常
			if (status.lowerStatus >> 2) {
				FSAIRobotInterface::set_bit(displayData->runStatus[i], 4, true);
			}
			// 上位机异常
			if (status.upperStatus > 0) {
				FSAIRobotInterface::set_bit(displayData->runStatus[i], 5, true);
			}

			// 下位机异常码
			displayData->LErrCode[i] = status.lowerStatus;
			// 上位机异常码
			displayData->UErrCode[i] = status.upperStatus;
		}


		// IO 状态

		// 触发线程刷新
		emit update_data(*displayData);

		// --- 循环执行标志置位，触发任务下发

		// 设置下次唤醒时间
		wakeUpTime += std::chrono::milliseconds(duration);
		// 休眠
		auto now = std::chrono::steady_clock::now();

		if ((now - wakeUpTime).count() > 0) {
			//std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(wakeUpTime - start).count() << std::endl;
			long long detTime = std::chrono::duration_cast<std::chrono::milliseconds>(now - wakeUpTime).count();
			wakeUpTime += std::chrono::milliseconds((detTime / duration + 1) * duration);
		}
		else
			std::this_thread::sleep_until(wakeUpTime);
	}

	return;
}

void Worker::switch_robot(int idx) {
	displayData->selectedRobot = idx;
}

void Worker::stopWork() {
	workerHealthy = false;
}


/**
* @brief  主窗口
*/
FSAIApp::FSAIApp() {
	// 资源初始化
	// 主窗口
	mainWindow = std::shared_ptr<MainWindow>(new MainWindow);
	// 全局配置窗口
	advanceWindow = std::shared_ptr<AdvanceConfigWindow>(new AdvanceConfigWindow);
	// 工艺窗口
	procedureWindow = std::shared_ptr<ProcedureWindow>(new ProcedureWindow);

	//QWidget * newWidget = new QWidget(mainWindow.get(), Qt::Tool);

	// 线程
	worker = new Worker;

	// 加载默认配置

	// 状态数据初始化
	//robotStatus = std::shared_ptr<RobotStatus>(new RobotStatus);

	// 显示主页
	mainWindow->show();

	// 默认连接
	ZController = std::shared_ptr<FSAIRobotInterface::Controller>(new FSAIRobotInterface::Controller);
	ZController->lazy_connect();

	// 申明机器人, 声明后在group中管理
	for (size_t i = 0; i < worker->displayData->robotNum; ++i) {
		std::shared_ptr<FSAIRobotInterface::RobotBase> robot;
		if (worker->displayData->interpAlgo == 0)
			robot = std::shared_ptr<FSAIRobotInterface::RobotBase>(new FSAIRobotInterface::ZMotionRobot);
		else if (worker->displayData->interpAlgo == 1)
			robot = std::shared_ptr<FSAIRobotInterface::RobotBase>(new FSAIRobotInterface::ZRVRobot);
		else
			robot = std::shared_ptr<FSAIRobotInterface::RobotBase>(new FSAIRobotInterface::FSAIRobot);

		robot->set_aliasId(i);
		robot->set_ZController(ZController);
		group.new_robot(robot);
	}

	// 设置公用轴
	group.set_shared_axis(6, { 0,1,2,3 });
	group.start_thread();

	// 连接信号和槽
	connect_slot();


	// 初始状态
	mainWindow->ui->radioButton->setChecked(true);

	// 默认工艺：无工艺
	std::map<int, std::vector<float>> proc;
	// 焊接参数
	Arc_WeldingParaItem weldCfg;
	weldCfg.Id = 0;
	auto weldPair = FSAIRobotInterface::serialize_Arc_WeldingParaItem(weldCfg);
	proc[weldPair.first] = weldPair.second;

	// 开启状态刷新线程
	worker->moveToThread(&workerThread);
	QObject::connect(&workerThread, &QThread::started, worker, &Worker::doWork);
	workerThread.start();

}

FSAIApp::~FSAIApp() {
	// 停止机器人Group
	group.stop();

	// 停止主界面刷新线程
	worker->stopWork();
	workerThread.terminate();
	workerThread.wait();
}



void FSAIApp::connect_slot() {

	/* ********************** 菜单栏 ********************** */
	QObject::connect(mainWindow->ui->actionConfig, &QAction::triggered, this, [&]() {
		// 传入参数
		advanceWindow->refresh_display_data(*(worker->displayData));
		advanceWindow->show();
		advanceWindow->activateWindow();
	});
	QObject::connect(mainWindow->ui->actionProcedure, &QAction::triggered, this, [&]() {
		
		procedureWindow->cmdIdx = -1;

		// 更新工艺参数界面
		display_procedure_data(-1, -1);

		// 传入参数
		procedureWindow->show();
		procedureWindow->activateWindow();
	});
	// 另存为工程
	QObject::connect(mainWindow->ui->actionSave_As, &QAction::triggered, this, [&]() {
		// 选择另存文件
		QString filename = QFileDialog::getSaveFileName(this, "Select project file", "./", "Json(*.json);;All files(*.*)");
		if (filename.isEmpty()) {
			mainWindow->ui->textBrowser->append("File not exists.");
			return;
		}
		// 中文路径转码
		QTextCodec * code = QTextCodec::codecForName("GB2312");
		std::string name = code->fromUnicode(filename).data();
		
		// 保存当前工程名称
		worker->displayData->projectName = name;
		mainWindow->ui->textBrowser->append("Project saved as \"" + QString::fromStdString(name) + "\"");

		save_project(name);
	});
	// 保存工程
	QObject::connect(mainWindow->ui->actionSave, &QAction::triggered, this, [&]() {
		std::string name = worker->displayData->projectName;
		mainWindow->ui->textBrowser->append("Project saved as \"" + QString::fromStdString(name) + "\"");

		save_project(name);
	});
	// 加载工程
	QObject::connect(mainWindow->ui->actionOpen, &QAction::triggered, this, [&]() {
		QString filename = QFileDialog::getOpenFileName(this, "Select project file", "./", "Json(*.json);;All files(*.*)");
		if (filename.isEmpty()) {
			mainWindow->ui->textBrowser->append("File not exists");
			return;
		}
		// 中文路径转码
		QTextCodec * code = QTextCodec::codecForName("GB2312");
		std::string name = code->fromUnicode(filename).data();

		//std::ifstream ifile("project/teach_point.json");
		std::ifstream ifile(name);
		if (!ifile) {
			mainWindow->ui->textBrowser->append("Open file failed");
			return;
		}

		nlohmann::json json;
		json << ifile;

		// 示教点位
		nlohmann::json teachPoint = json["teach_point"];

		for (size_t i = 0; i < worker->displayData->robotNum; ++i) {
			std::string idxStr = std::to_string(i);
			// 不包含机器人索引的字段
			if (!teachPoint.contains(idxStr))
				continue;
			worker->displayData->teachPoints[i] = teachPoint.at(idxStr).dump();
		}
		// 显示当前机器人点位
		display_teach_point(worker->displayData->teachPoints[worker->displayData->selectedRobot]);

		// 工艺参数
		nlohmann::json procedure = json["procedure"];
		for (size_t i = 0; i < procedure.size(); ++i) {
			// 第 i 个工艺
			nlohmann::json proc = procedure[std::to_string(i)];
			std::map<int, std::vector<float>> map;

			// 键值对转map
			for (auto& item : proc.items()) {
				int key = atol(item.key().c_str());
				std::vector<float> value = item.value().get<std::vector<float>>();

				map[key] = value;
			}

			// 保存工艺参数
			procedureWindow->procedure[i] = map;
		}

		mainWindow->ui->textBrowser->append("Project Loaded.");
	});

	// 设置窗口
	QObject::connect(advanceWindow->ui->pushButton_3, &QPushButton::released, this, [&]() {
		// 插补算法类型
		int curInterpAlgo = advanceWindow->ui->comboBox->currentIndex();
		if (worker->displayData->interpAlgo != curInterpAlgo) {
			for (size_t i = 0; i < worker->displayData->robotNum; ++i) {
				// 当前机器人ID
				int robotId = group.robotList[i]->get_robotId();
				int aliasId = group.robotList[i]->get_aliasId();

				// 切换算法类型
				std::shared_ptr<FSAIRobotInterface::RobotBase> robot;
				if (curInterpAlgo == 0) {
					robot = std::shared_ptr<FSAIRobotInterface::RobotBase>(new FSAIRobotInterface::ZMotionRobot);
				}
				else if (curInterpAlgo == 1) {
					robot = std::shared_ptr<FSAIRobotInterface::RobotBase>(new FSAIRobotInterface::ZRVRobot);
				}
				else if (curInterpAlgo == 2) {
					robot = std::shared_ptr<FSAIRobotInterface::RobotBase>(new FSAIRobotInterface::FSAIRobot);
				}

				robot->set_aliasId(aliasId);
				robot->set_ZController(ZController, robotId);
				// 注意共享指针的替换
				group.robotList[i] = robot;
			}

			mainWindow->ui->textBrowser->append("Switch InterpAlgo: " + advanceWindow->ui->comboBox->currentText());
		}

		advanceWindow->export_display_data(*(worker->displayData));

	});
	// 测试按钮1
	QObject::connect(advanceWindow->ui->pushButton_9, &QPushButton::released, this, [&]() {
		static bool streamOn = false;
		streamOn = !streamOn;

		// 开启缓存读取线程
		group.slave_buffer_stream(streamOn);
		QString log;

		if (streamOn) {
			// 下位机时间同步
			uint64_t masterStamp, slaveStamp;
			group.robotList[0]->synchronize_slave_buffer(masterStamp, slaveStamp);
			log = "Test 1: stream on " + QString::number(masterStamp) + ", " + QString::number(slaveStamp);
		}
		else {
			auto now = std::chrono::steady_clock::now();
			auto masterStamp = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
			log = "Test 1: stream off " + QString::number(masterStamp);
		}

		mainWindow->ui->textBrowser->append(log);
	});
	// 测试按钮2
	QObject::connect(advanceWindow->ui->pushButton_10, &QPushButton::released, this, [&]() {
		std::vector<motion::BufferUnit> buffer;
		group.robotList[0]->pop_slave_buffer(buffer, 1);


		for (size_t i = 0; i < buffer.size(); ++i) {
			QString dpos;
			for (auto& val : buffer[i].dpos) {
				dpos += QString::number(val) + ", ";
			}
			mainWindow->ui->textBrowser->append("Test 2: num " + QString::number(i) + ", " + QString::number(buffer[i].timeStamp) + ": " + dpos);
		}
	});

	// 工艺窗口
	// 切换工艺
	QObject::connect(procedureWindow->ui->comboBox_4, (void (QComboBox::*)(int)) &QComboBox::currentIndexChanged, this, [&](int procIdx) {
		// 更新工艺参数界面
		display_procedure_data(procIdx, -1);
	});
	// 保存新参数
	QObject::connect(procedureWindow->ui->buttonBox, &QDialogButtonBox::accepted, this, &FSAIApp::save_procedure_data);

	/* ********************** 监控页面 ********************** */
	// 状态刷新线程触发显示更新
	QObject::connect(worker, &Worker::update_data, mainWindow.get(), (void (MainWindow::*)(const MainWindowDisplayData&)) &MainWindow::update_display);


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
			int idx = worker->displayData->selectedRobot;
			group.robotList[idx]->jog_moving(type, i, -1, 1);
		});
		QObject::connect(mainWindow->jogMoveBtn[2 * i], &QPushButton::released, this, [&, i]() {
			int type = 0;
			for (size_t j = 0; j < mainWindow->jogTypeBtn.size(); ++j) {
				if (mainWindow->jogTypeBtn[j]->isChecked()) {
					type = j;
					break;
				}
			}
			int idx = worker->displayData->selectedRobot;
			group.robotList[idx]->jog_moving(type, i, 0, 0);
		});

		QObject::connect(mainWindow->jogMoveBtn[2 * i + 1], &QPushButton::pressed, this, [&, i]() {
			int type = 0;
			for (size_t j = 0; j < mainWindow->jogTypeBtn.size(); ++j) {
				if (mainWindow->jogTypeBtn[j]->isChecked()) {
					type = j;
					break;
				}
			}
			int idx = worker->displayData->selectedRobot;
			group.robotList[idx]->jog_moving(type, i, 1, 1);
		});
		QObject::connect(mainWindow->jogMoveBtn[2 * i + 1], &QPushButton::released, this, [&, i]() {
			int type = 0;
			for (size_t j = 0; j < mainWindow->jogTypeBtn.size(); ++j) {
				if (mainWindow->jogTypeBtn[j]->isChecked()) {
					type = j;
					break;
				}
			}
			int idx = worker->displayData->selectedRobot;
			group.robotList[idx]->jog_moving(type, i, 0, 0);
		});
	}


	/* ********************** 控制页面 ********************** */
	// 连接控制卡
	QObject::connect(mainWindow->ui->pushButton, &QPushButton::released, this, [&]() {
		// 若已连接则断开当前连接
		ZController->disconnect();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		QString address = mainWindow->ui->comboBox->currentText();
		int ret = 0;
		// 数字
		bool isNum = true;
		int cardId = address.toInt(&isNum);
		// IP 地址判断
		int ipType = validIPAddress(address.toStdString());

		// 自动连接
		if (address.isEmpty()) {
			ret = ZController->lazy_connect();
		}
		// IP 连接
		else if (ipType == 1) {
			ret = ZController->connect_eth(address.toStdString().c_str());
		}
		// 750 连接
		else if (address == "LOCAL") {
			ret = ZController->connect_pci(0, true, true);
		}
		// PCI 连接
		else if (isNum) {
			ret = ZController->connect_pci(cardId, false, true);
		}
		// 连接异常
		else {
			mainWindow->ui->textBrowser->append("Connection Address illegal.");
			ret = 1;
		}

		// 连接失败
		if (ret) {
			mainWindow->ui->textBrowser->append("Connect to controller failed");
			return;
		}

		mainWindow->ui->textBrowser->append("Connect to controller successful: " + address);
		group.robotList[0]->set_ZController(ZController, group.robotList[0]->get_robotId());

	});

	// 切换手自动模式
	QObject::connect(mainWindow->ui->checkBox_2, &QCheckBox::released, this, [&]() {
		std::vector<int> idxList = get_selected_robot_idx();
		int curIdx = worker->displayData->selectedRobot;
		// 目标模式
		int goalMode = 1 - FSAIRobotInterface::get_bit(worker->displayData->robotMode[curIdx], 0);

		for (auto& idx : idxList) {
			group.robotList[idx]->switch_auto(goalMode);
		}
	});

	// 切换选中机器人
	for (size_t i = 0; i < 4; ++i) {
		// 切换机器人
		QObject::connect(mainWindow->robotButton[i], &QPushButton::pressed, this, [&, i]() {
			if (i >= worker->displayData->robotNum) {
				mainWindow->ui->textBrowser->append("Exceed max robot num: " + QString::number(worker->displayData->robotNum));
				return;
			}

			// 保存记录点位信息
			std::vector<int> rowList(mainWindow->ui->tableWidget->rowCount(), 0);
			for (size_t i = 0; i < rowList.size(); ++i)
				rowList[i] += i;
			std::string ans;
			save_teach_point(rowList, ans);
			worker->displayData->teachPoints[worker->displayData->selectedRobot] = ans;
			//std::cout << std::setw(4) << nlohmann::json::parse(ans) << std::endl;

			worker->switch_robot(i);

			// 加载新机器人的点位信息
			display_teach_point(worker->displayData->teachPoints[i]);

			mainWindow->ui->textBrowser->append("Switch to robot " + QString::number(i));
		});
	}

	// 清空任务/报警
	QObject::connect(mainWindow->ui->pushButton_10, &QPushButton::pressed, this, [&]() {
		std::vector<int> idxList = get_selected_robot_idx();

		// 清除任务
		for (auto& idx : idxList) {
			//group.robotList[idx]->task_stop();
			group.robot_group_clear_task(idx);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		// 使能
		for (auto& idx : idxList) {
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			group.robotList[idx]->switch_enable(true);
		}
	});

	// 重启机器人
	QObject::connect(mainWindow->ui->pushButton_11, &QPushButton::pressed, this, [&]() {
		std::vector<int> idxList = get_selected_robot_idx();

		for (auto& idx : idxList) {
			if (worker->displayData->interpAlgo == 0) {
				group.robotList[idx]->reboot("./ctr/ZMotionRobot.zar", 0);
			}
			else if (worker->displayData->interpAlgo == 1) {
				group.robotList[idx]->reboot("./ctr/ZRVRobot.zar", 0);
			}
			else if (worker->displayData->interpAlgo == 2) {
				group.robotList[idx]->reboot("./ctr/FSAIRobot.zar", 0);
			}
		}
	});

	// 系统急停
	QObject::connect(mainWindow->ui->pushButton_34, &QPushButton::pressed, this, [&]() {
		for (auto& robot : group.robotList) {
			robot->emergency_stop();
		}
	});

	// 速度设置
	QObject::connect(mainWindow->ui->horizontalSlider, &QSlider::sliderReleased, this, [&]() {
		// 当前速度
		float speed = mainWindow->ui->spinBox->value();
		// 当前机器人
		int idx = worker->displayData->selectedRobot;

		group.robotList[idx]->set_manual_speed(speed);
		mainWindow->ui->textBrowser->append("set speed ratio: " + QString::number(speed) + "%");
	});
	QObject::connect(mainWindow->ui->spinBox, &QSpinBox::editingFinished, this, [&]() {
		// 当前速度
		float speed = mainWindow->ui->spinBox->value();
		// 当前机器人
		int idx = worker->displayData->selectedRobot;

		group.robotList[idx]->set_manual_speed(speed);
		mainWindow->ui->textBrowser->append("set speed ratio: " + QString::number(speed) + "%");
	});

	/* ********************** 示教页面 ********************** */
	// 开始
	QObject::connect(mainWindow->ui->pushButton_4, &QPushButton::pressed, this, [&]() {
		std::vector<int> idxList = get_selected_robot_idx();
		
		std::vector<DiscreteTrajectory> traj(4);
		for (auto& idx : idxList) {
			if (idx == worker->displayData->selectedRobot) {
				// 示教点
				std::string teachPointStr;
				std::vector<int> rowList(mainWindow->ui->tableWidget->rowCount(), 0);
				for (size_t i = 0; i < rowList.size(); ++i)
					rowList[i] += i;
				save_teach_point(rowList, teachPointStr);

				for (size_t i = 0; i < rowList.size(); ++i)
					teach_point_to_trajectory(teachPointStr, i, traj[idx]);
			}
			else {
				nlohmann::json teachPoint;
				if (!worker->displayData->teachPoints[idx].empty()) {
					teachPoint = nlohmann::json::parse(worker->displayData->teachPoints[idx]);
				}

				for (size_t i = 0; i < teachPoint.size(); ++i)
					teach_point_to_trajectory(worker->displayData->teachPoints[idx], i, traj[idx]);
			}
		}

		for (auto& idx : idxList) {
			int ret = group.robotList[idx]->push_new_trajectory(traj[idx]);
			if (ret)
				mainWindow->ui->textBrowser->append("send traj error: " + QString::number(ret));
		}
	});
	// 执行当前
	QObject::connect(mainWindow->ui->pushButton_7, &QPushButton::pressed, this, [&]() {

		// 当前选中示教点
		int row = mainWindow->ui->tableWidget->currentRow();

		if (row < 0) {
			mainWindow->ui->textBrowser->append("Please selecte at least one teach point.");
			return;
		}

		// 当前选中点为圆弧终点
		QWidget* widget = mainWindow->ui->tableWidget->cellWidget(row, 1);
		int moveType = ((QComboBox*)widget)->currentIndex();
		if (moveType < 0) {
			mainWindow->ui->textBrowser->append("Please selecte mid point of arc trajectory");
			return;
		}

		// 示教点
		DiscreteTrajectory traj;
		std::string teachPointStr;
		save_teach_point({ row }, teachPointStr);

		teach_point_to_trajectory(teachPointStr, { row }, traj);

		int ret = group.robotList[worker->displayData->selectedRobot]->push_new_trajectory(traj);
		if (ret)
			mainWindow->ui->textBrowser->append("send traj error: " + QString::number(ret));

	});
	// 暂停/继续
	QObject::connect(mainWindow->ui->pushButton_5, &QPushButton::pressed, this, [&]() {
		std::vector<int> idxList = get_selected_robot_idx();
		int curIdx = worker->displayData->selectedRobot;

		for (auto& idx : idxList) {
			// 当前处于暂停状态
			if (FSAIRobotInterface::get_bit(worker->displayData->runStatus[curIdx], 3)) {
				group.robot_group_resume(idx);
			}
			else {
				group.robot_group_pause(idx);
			}
		}
	});
	// 停止
	QObject::connect(mainWindow->ui->pushButton_6, &QPushButton::pressed, this, [&]() {
		std::vector<int> idxList = get_selected_robot_idx();

		for (auto& idx : idxList) {
			group.robot_group_stop(idx);
		}
	});

	// 记录示教点
	QObject::connect(mainWindow->ui->pushButton_2, &QPushButton::pressed, this, [&]() {
		// 当前行
		int row = mainWindow->ui->tableWidget->currentRow();

		// 设置当前行默认运动参数
		auto moveCfg = FSAIRobotInterface::deserialize_Move_Config(mainWindow->moveCfg[row]);
		moveCfg.speed = 10;
		moveCfg.smooth = -1;
		auto movePair = FSAIRobotInterface::serialize_Move_Config(moveCfg);
		mainWindow->moveCfg[row][movePair.first] = movePair.second;
		// 设置当前默认协同参数
		FSAIRobotInterface::Sync_Config syncCfg;
		syncCfg.Id = 0;
		auto syncPair = FSAIRobotInterface::serialize_Sync_Config(syncCfg);
		mainWindow->moveCfg[row][syncPair.first] = syncPair.second;

		insert_teach_point_config_button({ row });
	});


	/* ********************** 日志页面 ********************** */
	// 按键下发指令
	QObject::connect(mainWindow->ui->pushButton_8, &QPushButton::pressed, this, [&]() {
		auto cmd = mainWindow->ui->lineEdit->text();

		// 下发指令，获取返回值
		std::string ack;
		std::string cmdString = cmd.toStdString();
		int ret = group.robotList[0]->send_command(cmd.toStdString(), ack, 0);

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
		int ret = group.robotList[0]->send_command(cmd.toStdString(), ack, 0);
		if (ret)
			mainWindow->ui->textBrowser->append("error return: " + QString::number(ret));
		else
			mainWindow->ui->textBrowser->append(QString::fromStdString(ack));
	});

}


std::vector<float> FSAIApp::read_list_from_tableWidget(const QTableWidget* table, int row, const std::vector<int>& idxList) {
	std::vector<float> ans;
	for (auto& idx : idxList) {
		QString str = table->item(row, idx)->text();
		QStringList strList = str.split(",");

		for (auto& word : strList) {
			ans.push_back(word.toFloat());
		}
	}
	return ans;
}

std::vector<int> FSAIApp::get_selected_robot_idx() {
	std::vector<int> idxList;
	// 操作所有机器人
	if (mainWindow->ui->checkBox_19->isChecked()) {
		for (size_t i = 0; i < worker->displayData->robotNum; ++i) {
			idxList.push_back(i);
		}
	}
	// 当前机器人
	else {
		idxList.push_back(worker->displayData->selectedRobot);
	}
	return idxList;
}

void FSAIApp::display_procedure_data(int procIdx, int cmdIdx) {

	// 未指定工艺号，按上次修改工艺显示
	if (procIdx < 0) {
		procIdx = procedureWindow->ui->comboBox_4->currentIndex();
	}

	// 未指定行号，仅修改工艺
	if (cmdIdx >= 0) {
		auto moveCfg = FSAIRobotInterface::deserialize_Move_Config(mainWindow->moveCfg[cmdIdx]);
		// 平滑度
		procedureWindow->ui->spinBox_11->setValue(moveCfg.smooth);

		// 协同参数
		FSAIRobotInterface::Sync_Config syncCfg = FSAIRobotInterface::deserialize_Sync_Config(mainWindow->moveCfg[cmdIdx]);
		procedureWindow->ui->groupBox_12->setChecked(syncCfg.Id > 0);
		std::vector<int> syncRobot;
		if ((syncCfg.Id > 0 || syncCfg.map.size() > 0) && (syncCfg.map.begin() != syncCfg.map.end())) {
			auto firstSync = *(syncCfg.map.begin());
			procedureWindow->ui->spinBox_22->setValue(firstSync.first);
			procedureWindow->ui->spinBox_23->setValue(firstSync.second.begin()->second);
			for (auto& pair : firstSync.second) {
				syncRobot.push_back(pair.first);
			}
			QString syncRobotStr;
			for (int i = 0; i < syncRobot.size() - 1; ++i) {
				syncRobotStr += QString::number(syncRobot[i]) + ", ";
			}
			syncRobotStr += QString::number(syncRobot.back());
			procedureWindow->ui->lineEdit_2->setText(syncRobotStr);
		}
		else {
			procedureWindow->ui->spinBox_22->setValue(-1);
			procedureWindow->ui->spinBox_23->setValue(0);
			procedureWindow->ui->lineEdit_2->setText("");
		}

		// 可编辑工艺和运动参数
		procedureWindow->ui->groupBox_9->setDisabled(false);
		procedureWindow->ui->groupBox_12->setDisabled(false);

		// 设置不可切换工艺
		procedureWindow->ui->comboBox_4->setDisabled(true);

	}
	else {
		procedureWindow->ui->spinBox_11->setValue(-1);

		procedureWindow->ui->spinBox_22->setValue(-1);
		procedureWindow->ui->spinBox_23->setValue(0);
		procedureWindow->ui->lineEdit_2->setText("");

		// 仅修改工艺，不修改运动参数
		procedureWindow->ui->groupBox_9->setDisabled(true);
		procedureWindow->ui->groupBox_12->setDisabled(true);

		// 设置可切换工艺
		procedureWindow->ui->comboBox_4->setDisabled(false);

	}

	// 焊接参数
	Arc_WeldingParaItem weldCfg = FSAIRobotInterface::deserialize_Arc_WeldingParaItem(procedureWindow->procedure[procIdx]);

	// 直流 / 脉冲
	int workPattern = (weldCfg.WeldingWorkMode >> 2) % 2;
	// 一元 / 分别
	int voltageMode = (weldCfg.WeldingWorkMode >> 4) % 2;
	procedureWindow->ui->comboBox->setCurrentIndex(workPattern);
	procedureWindow->ui->comboBox_2->setCurrentIndex(voltageMode);

	// 焊接参数 0
	procedureWindow->ui->groupBox->setChecked(weldCfg.Id);                    // 0 起弧标志
	procedureWindow->ui->spinBox_12->setValue(weldCfg.WeldingCrt_Spd);        // 1 焊接电流
	procedureWindow->ui->spinBox->setValue(weldCfg.WeldingVtg_Strth);         // 2 焊接电压
	procedureWindow->ui->spinBox->setValue(weldCfg.VtgUniCorrection);         // 4 焊接电压修正值
     
	// 起弧参数 5
	procedureWindow->ui->spinBox_13->setValue(weldCfg.ArcOnCrt_Spd);          // 1 起弧电流
	procedureWindow->ui->spinBox_14->setValue(weldCfg.ArcOnVtg_Strth);	      // 2 起弧电压
	procedureWindow->ui->spinBox_2->setValue(weldCfg.ArcOnTime);	          // 3 起弧时间
	procedureWindow->ui->spinBox_14->setValue(weldCfg.ArcOnVtg_Correction);   // 4 起弧电压修正值
	//weldCfg.ArcOnBlowTime = param[num++];	                                  // 5 引气时间						   

	// 收弧参数 11
	procedureWindow->ui->spinBox_15->setValue(weldCfg.ArcOffCrt_Spd);         // 1 收弧电流
	procedureWindow->ui->spinBox_16->setValue(weldCfg.ArcOffVtg_Strth);       // 2 收弧电压
	procedureWindow->ui->spinBox_3->setValue(weldCfg.ArcOffTime);             // 3 收弧时间
	procedureWindow->ui->spinBox_16->setValue(weldCfg.ArcOffVtg_Correction);  // 4 收弧电压修正值
	//weldCfg.ArcOffBlowTime = param[num++];  // 5 收气时间


	// 摆焊参数
	Weave waveCfg = FSAIRobotInterface::deserialize_Weave(procedureWindow->procedure[procIdx]);
	procedureWindow->ui->groupBox_7->setChecked(waveCfg.Id);
	procedureWindow->ui->comboBox_3->setCurrentIndex(waveCfg.Shape);
	procedureWindow->ui->doubleSpinBox_7->setValue(waveCfg.Freq);
	procedureWindow->ui->doubleSpinBox_8->setValue(waveCfg.LeftWidth);
	procedureWindow->ui->doubleSpinBox_9->setValue(waveCfg.RightWidth);
	procedureWindow->ui->radioButton->setChecked(waveCfg.Dwell_type);
	procedureWindow->ui->spinBox_4->setValue(waveCfg.Dwell_left);
	procedureWindow->ui->spinBox_5->setValue(waveCfg.Dwell_right);
	procedureWindow->ui->spinBox_6->setValue(waveCfg.Dwell_center);
	// 三角摆
	procedureWindow->ui->spinBox_9->setValue(waveCfg.Length);
	procedureWindow->ui->spinBox_10->setValue(waveCfg.Bias);
	procedureWindow->ui->spinBox_8->setValue(waveCfg.Angle_Ltype_top);
	procedureWindow->ui->spinBox_7->setValue(waveCfg.Angle_Ltype_btm);


	// 跟踪参数
	Track trackCfg = FSAIRobotInterface::deserialize_Track(procedureWindow->procedure[procIdx]);
	procedureWindow->ui->groupBox_3->setChecked(trackCfg.Id);
	// 左右跟踪参数
	procedureWindow->ui->groupBox_2->setChecked(trackCfg.Lr_enable);
	procedureWindow->ui->doubleSpinBox_11->setValue(trackCfg.Lr_offset);
	procedureWindow->ui->doubleSpinBox_10->setValue(trackCfg.Lr_gain);
	procedureWindow->ui->doubleSpinBox->setValue(trackCfg.Lr_minCompensation);
	procedureWindow->ui->doubleSpinBox_12->setValue(trackCfg.Lr_maxCompensation);
	// 上下跟踪参数
	procedureWindow->ui->groupBox_8->setChecked(trackCfg.Ud_enable);


	// 运动参数
	FSAIRobotInterface::Move_Config moveCfg = FSAIRobotInterface::deserialize_Move_Config(procedureWindow->procedure[procIdx]);
	procedureWindow->ui->doubleSpinBox_6->setValue(moveCfg.speed);


	// 缓冲运动
	FSAIRobotInterface::Move_Action moveAct = FSAIRobotInterface::deserialize_Move_Action(procedureWindow->procedure[procIdx]);

}

void FSAIApp::save_procedure_data() {

	// 当前工艺号
	int procIdx = procedureWindow->ui->comboBox_4->currentIndex();

	std::map<int, std::vector<float>> proc;

	// 焊接参数
	Arc_WeldingParaItem weldCfg;
	// 焊接模式
	int workPattern = procedureWindow->ui->comboBox->currentIndex();
	int voltageMode = procedureWindow->ui->comboBox_2->currentIndex();
	int weldWorkMode = 2;
	// 直流 / 脉冲
	weldWorkMode += workPattern ? 4 : 0;
	// 一元 / 分别
	weldWorkMode += voltageMode ? 16 : 0;

	// 焊接参数 0
	weldCfg.Id = procedureWindow->ui->groupBox->isChecked();                   // 0 起弧标志
	weldCfg.WeldingCrt_Spd = procedureWindow->ui->spinBox_12->value();         // 1 焊接电流
	weldCfg.WeldingVtg_Strth = procedureWindow->ui->spinBox->value();          // 2 焊接电压
	weldCfg.WeldingWorkMode = weldWorkMode;                                    // 3 焊接工作模式
	weldCfg.VtgUniCorrection = procedureWindow->ui->spinBox->value();          // 4 焊接电压修正值

	// 起弧参数 5
	weldCfg.ArcOnWorkMode = weldWorkMode;                                      // 0 起弧模式
	weldCfg.ArcOnCrt_Spd = procedureWindow->ui->spinBox_13->value();	       // 1 起弧电流
	weldCfg.ArcOnVtg_Strth = procedureWindow->ui->spinBox_14->value();	       // 2 起弧电压
	weldCfg.ArcOnTime = procedureWindow->ui->spinBox_2->value();	           // 3 起弧时间
	weldCfg.ArcOnVtg_Correction = procedureWindow->ui->spinBox_14->value();    // 4 起弧电压修正值
	//weldCfg.ArcOnBlowTime = param[num++];	 // 5 引气时间						   

	// 收弧参数 11
	weldCfg.ArcOffWorkMode = weldWorkMode;                                     // 0 收弧模式
	weldCfg.ArcOffCrt_Spd = procedureWindow->ui->spinBox_15->value();          // 1 收弧电流
	weldCfg.ArcOffVtg_Strth = procedureWindow->ui->spinBox_16->value();        // 2 收弧电压
	weldCfg.ArcOffTime = procedureWindow->ui->spinBox_3->value();              // 3 收弧时间
	weldCfg.ArcOffVtg_Correction = procedureWindow->ui->spinBox_16->value();   // 4 收弧电压修正值
	//weldCfg.ArcOffBlowTime = param[num++];  // 5 收气时间
	auto weldPair = FSAIRobotInterface::serialize_Arc_WeldingParaItem(weldCfg);
	proc[weldPair.first] = weldPair.second;

	// 摆焊参数
	Weave waveCfg;
	waveCfg.Id = procedureWindow->ui->groupBox_7->isChecked();
	waveCfg.Shape = procedureWindow->ui->comboBox_3->currentIndex();
	waveCfg.Freq = procedureWindow->ui->doubleSpinBox_7->value();
	waveCfg.LeftWidth = procedureWindow->ui->doubleSpinBox_8->value();
	waveCfg.RightWidth = procedureWindow->ui->doubleSpinBox_9->value();
	waveCfg.Dwell_type = procedureWindow->ui->radioButton->isChecked();
	waveCfg.Dwell_left = procedureWindow->ui->spinBox_4->value();
	waveCfg.Dwell_right = procedureWindow->ui->spinBox_5->value();
	waveCfg.Dwell_center = procedureWindow->ui->spinBox_6->value();
	// 三角摆
	waveCfg.Length = procedureWindow->ui->spinBox_9->value();
	waveCfg.Bias = procedureWindow->ui->spinBox_10->value();
	waveCfg.Angle_Ltype_top = procedureWindow->ui->spinBox_8->value();
	waveCfg.Angle_Ltype_btm = procedureWindow->ui->spinBox_7->value();

	auto wavePair = FSAIRobotInterface::serialize_Weave(waveCfg);
	proc[wavePair.first] = wavePair.second;


	// 跟踪参数
	Track trackCfg;
	trackCfg.Id = procedureWindow->ui->groupBox_3->isChecked();
	// 左右跟踪参数
	trackCfg.Lr_enable = procedureWindow->ui->groupBox_2->isChecked();
	trackCfg.Lr_offset = procedureWindow->ui->doubleSpinBox_11->value();
	trackCfg.Lr_gain = procedureWindow->ui->doubleSpinBox_10->value();
	trackCfg.Lr_minCompensation = procedureWindow->ui->doubleSpinBox->value();
	trackCfg.Lr_maxCompensation = procedureWindow->ui->doubleSpinBox_12->value();
	// 上下跟踪参数
	trackCfg.Ud_enable = procedureWindow->ui->groupBox_8->isChecked();

	auto trackPair = FSAIRobotInterface::serialize_Track(trackCfg);
	proc[trackPair.first] = trackPair.second;


	// 运动参数
	FSAIRobotInterface::Move_Config moveCfg;
	moveCfg.speed = procedureWindow->ui->doubleSpinBox_6->value();

	auto movePair = FSAIRobotInterface::serialize_Move_Config(moveCfg);
	proc[movePair.first] = movePair.second;



	// 保存工艺参数
	procedureWindow->procedure[procIdx] = proc;


	// 保存轨迹参数
	if (procedureWindow->cmdIdx >= 0) {
		// 运动参数
		FSAIRobotInterface::Move_Config moveCfg;
		moveCfg.smooth = procedureWindow->ui->spinBox_11->value();

		auto pair = FSAIRobotInterface::serialize_Move_Config(moveCfg);
		mainWindow->moveCfg[procedureWindow->cmdIdx][pair.first] = pair.second;

		// 协同参数
		FSAIRobotInterface::Sync_Config syncCfg;
		syncCfg.Id = procedureWindow->ui->groupBox_12->isChecked();
		int syncType = procedureWindow->ui->spinBox_22->value();
		int syncId = procedureWindow->ui->spinBox_23->value();
		QString str = procedureWindow->ui->lineEdit_2->text();
		QStringList strList = str.split(",");
		for (auto& word : strList) {
			bool ok;
			int syncRobot = word.toInt(&ok);
			if (ok)
				syncCfg.add_sync_item(syncType, syncRobot, syncId);
		}

		auto syncPair = FSAIRobotInterface::serialize_Sync_Config(syncCfg);
		mainWindow->moveCfg[procedureWindow->cmdIdx][syncPair.first] = syncPair.second;
	}

}


int FSAIApp::insert_teach_point_config_button(const std::vector<int>& idxList) {

	for (auto& row : idxList) {
		QPushButton* trajCfgBox = new QPushButton();
		//trajCfgBox->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
		//trajCfgBox->setText("...");
		mainWindow->ui->tableWidget->setCellWidget(row, 7, trajCfgBox);

		// 设置按钮
		//QPushButton *btn = (QPushButton *)(mainWindow->ui->tableWidget->cellWidget(row, 7));

		// 绑定打开工艺参数
		connect(trajCfgBox, &QPushButton::released, this, [&]() {

			// 当前行号
			int tRow = mainWindow->ui->tableWidget->currentRow();
			procedureWindow->cmdIdx = tRow;

			// 当前选中工艺号
			QWidget* widget = mainWindow->ui->tableWidget->cellWidget(tRow, 2);
			int procIdx = ((QSpinBox*)widget)->value();
			procedureWindow->ui->comboBox_4->setCurrentIndex(procIdx);

			// 更新工艺参数界面
			display_procedure_data(procIdx, tRow);

			// 显示窗口
			procedureWindow->show();
			procedureWindow->activateWindow();

		});

		// 轨迹类型
		QComboBox* typeComboBox = (QComboBox*)(mainWindow->ui->tableWidget->cellWidget(row, 1));
		QObject::connect(typeComboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, [&](int idx) {
			int row = mainWindow->ui->tableWidget->currentRow();
			// 下一条运动类型为圆弧终点
			if (idx == 2 && row + 1 < mainWindow->ui->tableWidget->rowCount()) {
				QWidget* preItem = mainWindow->ui->tableWidget->cellWidget(row + 1, 1);
				((QComboBox*)preItem)->setCurrentIndex(-1);
			}
			// 如果上一条运动为圆弧中间点，则当前类型为圆弧终点
			if (row > 0) {
				QWidget* preItem = mainWindow->ui->tableWidget->cellWidget(row - 1, 1);
				if (((QComboBox*)preItem)->currentIndex() == 2) {
					QWidget* preItem = mainWindow->ui->tableWidget->cellWidget(row, 1);
					((QComboBox*)preItem)->setCurrentIndex(-1);
				}
			}
			// 没有下一条轨迹则无法设置为圆弧中间点
			if (row + 1 >= mainWindow->ui->tableWidget->rowCount()) {
				QWidget* curItem = mainWindow->ui->tableWidget->cellWidget(row, 1);
				if (((QComboBox*)curItem)->currentIndex() == 2) {
					mainWindow->ui->textBrowser->append("Record end point of arc trajectory first");
					((QComboBox*)curItem)->setCurrentIndex(0);
				}
			}
		});
	}
	return 0;
}

// 界面示教点转化为 json 字符串
int FSAIApp::save_teach_point(const std::vector<int> rowList, std::string& result) {

	nlohmann::json teachPoint;

	for (auto& row : rowList) {
		if (row < 0 || row >= mainWindow->ui->tableWidget->rowCount()) {
			mainWindow->ui->textBrowser->append("Selected row not exists.");
			continue;
		}

		// 点位序号
		teachPoint[row]["Seq"] = mainWindow->ui->tableWidget->item(row, 0)->text().toInt();

		// 运动类型
		QWidget* widget = mainWindow->ui->tableWidget->cellWidget(row, 1);
		int moveType = ((QComboBox*)widget)->currentIndex();
		teachPoint[row]["MType"] = moveType;

		// 工艺号
		widget = mainWindow->ui->tableWidget->cellWidget(row, 2);
		int procIdx = ((QSpinBox*)widget)->value();
		teachPoint[row]["ProcIdx"] = procIdx;

		// 获取点位
		auto value = read_list_from_tableWidget(mainWindow->ui->tableWidget, row, { 3 });
		teachPoint[row]["JPos"] = value;

		value = read_list_from_tableWidget(mainWindow->ui->tableWidget, row, { 4 });
		teachPoint[row]["CPos"] = value;

		value = read_list_from_tableWidget(mainWindow->ui->tableWidget, row, { 5 });
		teachPoint[row]["External"] = value;

		//auto moveCfg = FSAIRobotInterface::deserialize_Move_Config(moveCfgMap);
		//cellItem = new QTableWidgetItem(QString::number(moveCfg.speed));
		//mainWindow->ui->tableWidget->setItem(row, 6, cellItem);
		// 速度
		teachPoint[row]["Speed"] = mainWindow->ui->tableWidget->item(row, 6)->text().toFloat();
		// 平滑度

		// 轨迹参数
		for (auto& cfg : mainWindow->moveCfg[row]) {
			teachPoint[row]["MoveConfig"][std::to_string(cfg.first)] = cfg.second;
		}

	}

	// 写入字符串
	result = teachPoint.dump();

	return 0;
}

// json 字符串转化为界面示教点
int FSAIApp::display_teach_point(const std::string& teachPointStr) {

	nlohmann::json teachPoint;
	if (!teachPointStr.empty()) {
		teachPoint = nlohmann::json::parse(teachPointStr);
	}

	// 删除点位
	mainWindow->moveCfg.clear();
	mainWindow->ui->tableWidget->setRowCount(teachPoint.size());
	mainWindow->moveCfg.resize(teachPoint.size());

	// 记录点位更新到当前界面
	for (auto& item : teachPoint.items()) {
		int row = atol(item.key().c_str());
		// 序号
		QTableWidgetItem* seqItem = new QTableWidgetItem(QString::number(row));
		seqItem->setFlags(seqItem->flags() & (~Qt::ItemIsEditable));
		mainWindow->ui->tableWidget->setItem(row, 0, seqItem);

		// 轨迹类型
		QComboBox* typeComboBox = new QComboBox();
		typeComboBox->addItem("     J");
		typeComboBox->addItem("     L");
		typeComboBox->addItem("     C");
		typeComboBox->setCurrentIndex(item.value()["MType"].get<int>());
		mainWindow->ui->tableWidget->setCellWidget(row, 1, typeComboBox);

		// 工艺号
		QSpinBox* procedureBox = new QSpinBox();
		procedureBox->setMaximum(10);
		procedureBox->setPrefix("Proc. ");
		procedureBox->setButtonSymbols(QSpinBox::NoButtons);
		procedureBox->setAlignment(Qt::AlignHCenter);
		procedureBox->setValue(item.value()["ProcIdx"].get<int>());
		mainWindow->ui->tableWidget->setCellWidget(row, 2, procedureBox);

		// 读取当前空间位置
		std::vector<float> pos = item.value()["JPos"].get<std::vector<float>>();
		QString posStr = QString::number(pos[0]);
		for (size_t i = 1; i < pos.size(); ++i) {
			posStr += ", " + QString::number(pos[i]);
		}
		QTableWidgetItem* cellItem = new QTableWidgetItem(posStr);
		mainWindow->ui->tableWidget->setItem(row, 3, cellItem);

		// 关节位置
		pos = (item.value()["CPos"]).get<std::vector<float>>();
		posStr = QString::number(pos[0]);
		for (size_t i = 1; i < pos.size(); ++i) {
			posStr += ", " + QString::number(pos[i]);
		}
		cellItem = new QTableWidgetItem(posStr);
		mainWindow->ui->tableWidget->setItem(row, 4, cellItem);

		// 附加轴位置
		pos = (item.value()["External"]).get<std::vector<float>>();
		posStr = QString::number(pos[0]);
		for (size_t i = 1; i < pos.size(); ++i) {
			posStr += ", " + QString::number(pos[i]);
		}
		cellItem = new QTableWidgetItem(posStr);
		mainWindow->ui->tableWidget->setItem(row, 5, cellItem);

		// 速度
		cellItem = new QTableWidgetItem(QString::number(item.value()["Speed"].get<float>()));
		mainWindow->ui->tableWidget->setItem(row, 6, cellItem);

		// 轨迹参数
		std::map<int, std::vector<float>> moveCfgMap;
		for (auto& cfg : item.value()["MoveConfig"].items()) {
			moveCfgMap[atol(cfg.key().c_str())] = cfg.value().get<std::vector<float>>();
		}
		mainWindow->moveCfg[row] = moveCfgMap;

		// 轨迹参数设置按钮
		insert_teach_point_config_button({ row });
	}

	return 0;
}

// json 字符串转化为轨迹类
int FSAIApp::teach_point_to_trajectory(const std::string teachPointStr, int row, DiscreteTrajectory& traj) {
	if (row < 0) {
		//mainWindow->ui->textBrowser->append("Teach point not selected.");
		return -1;
	}

	nlohmann::json teachPoint;
	if (!teachPointStr.empty()) {
		teachPoint = nlohmann::json::parse(teachPointStr);
	}
	// 传入轨迹为空
	else {
		return -1;
	}

	// 当前行转换为轨迹
	TrajectoryConfig trajCfg;

	// 运动类型
	int moveType = teachPoint[row]["MType"].get<int>();

	// 工艺号
	int procIdx = teachPoint[row]["ProcIdx"].get<int>();

	std::map<int, std::vector<float>> moveCfgMap;
	for (auto& cfg : teachPoint[row]["MoveConfig"].items()) {
		moveCfgMap[atol(cfg.key().c_str())] = cfg.value().get<std::vector<float>>();
	}
	auto moveCfg = FSAIRobotInterface::deserialize_Move_Config(moveCfgMap);
	// 平滑度
	trajCfg.smooth = moveCfg.smooth;
	// 速度
	float speed = teachPoint[row]["Speed"].get<float>();

	// 协同号
	auto syncCfg = FSAIRobotInterface::deserialize_Sync_Config(moveCfgMap);
	trajCfg.add_appendix(FSAIRobotInterface::serialize_Sync_Config(syncCfg));


	// 关节运动
	if (moveType == 0) {
		// 获取点位
		std::vector<float> dpos = teachPoint[row]["JPos"].get<std::vector<float>>();
		std::vector<float> epos = teachPoint[row]["External"].get<std::vector<float>>();
		dpos.insert(dpos.end(), epos.begin(), epos.end());

		trajCfg.speed = speed > 100 ? 100 : speed;
		traj.moveJABS(dpos, trajCfg);
	}
	// 忽略圆弧运动终点
	else if (moveType < 0) {
		return -1;
	}
	// 空间运动
	else {

		// 附加参数
		for (auto& cfg : procedureWindow->procedure[procIdx]) {
			trajCfg.add_appendix(cfg);
		}

		// 获取工艺参数
		Arc_WeldingParaItem weldCfg = FSAIRobotInterface::deserialize_Arc_WeldingParaItem(trajCfg.appendix);
		// 不允许起弧
		if (!mainWindow->ui->checkBox->isChecked()) {
			// 修改工艺参数起弧标志位
			weldCfg.Id = 0;
			trajCfg.add_appendix(FSAIRobotInterface::serialize_Arc_WeldingParaItem(weldCfg));
		}
		// 允许起弧
		else if (weldCfg.Id > 0) {
			FSAIRobotInterface::Move_Action moveAct = FSAIRobotInterface::deserialize_Move_Action(trajCfg.appendix);

			bool isBeg = true;
			// 当前行不是第一行
			if (row > 0) {
				// 获取前一条轨迹工艺参数
				int preProcIdx = teachPoint[row - 1]["ProcIdx"].get<int>();
				Arc_WeldingParaItem preWeldCfg = FSAIRobotInterface::deserialize_Arc_WeldingParaItem(procedureWindow->procedure[preProcIdx]);
				isBeg = preWeldCfg.Id <= 0;
			}

			// 焊接起点，起弧动作默认加在动作最后
			if (isBeg) {
				moveAct.actionBefore.push_back({ 2, FSAIRobotInterface::serialize_Arc_WeldingParaItem(weldCfg).second });
			}

			bool isEnd = true;
			int nextRow = (moveType == 2) ? row + 2 : row + 1;
			if (nextRow < teachPoint.size()) {
				// 获取前一条轨迹工艺参数
				int nextProcIdx = teachPoint[nextRow]["ProcIdx"].get<int>();
				Arc_WeldingParaItem nextWeldCfg = FSAIRobotInterface::deserialize_Arc_WeldingParaItem(procedureWindow->procedure[nextProcIdx]);
				isEnd = nextWeldCfg.Id <= 0;
			}

			// 添加息弧动作
			if (isEnd) {
				moveAct.actionAfter.push_back({ 3, FSAIRobotInterface::serialize_Arc_WeldingParaItem(weldCfg).second });
			}

			trajCfg.add_appendix(FSAIRobotInterface::serialize_Move_Action(moveAct));
		}

		// 非默认工艺，使用工艺指定的焊接速度
		trajCfg.speed = procIdx > 0 ? FSAIRobotInterface::deserialize_Move_Config(trajCfg.appendix).speed : speed;

		// 获取点位
		std::vector<float> dpos = teachPoint[row]["CPos"].get<std::vector<float>>();
		std::vector<float> epos = teachPoint[row]["External"].get<std::vector<float>>();
		dpos.insert(dpos.end(), epos.begin(), epos.end());
		// 直线运动
		if (moveType == 1) {
			traj.moveLABS(dpos, trajCfg);
		}
		// 圆弧运动
		else {
			// 没有下一个点
			if (row + 1 >= teachPoint.size()) {
				mainWindow->ui->textBrowser->append("No end point of arc trajectory: " + QString::number(row));
				return -1;
			}

			// 下一个点不是圆弧终点
			if (teachPoint[row + 1]["MType"].get<int>() >= 0) {
				mainWindow->ui->textBrowser->append("No end point of arc trajectory: " + QString::number(row));
				return -1;
			}

			// 获取终点位置
			std::vector<float> endPos = teachPoint[row + 1]["CPos"].get<std::vector<float>>();
			endPos.insert(endPos.end(), epos.begin(), epos.end());

			traj.moveCABS(dpos, endPos, trajCfg);
		}

	}

	return 0;
}

// 保存工程
int FSAIApp::save_project(std::string fileName) {

	std::ofstream ofile(fileName);
	if (!ofile) {
		mainWindow->ui->textBrowser->append("Open file failed");
		return -1;
	}

	// 按插入顺序保存
	nlohmann::ordered_json json;

	// 工程版本号
	json["version"] = 250916;

	// 工程名称

	// 保存每个机器人的示教点
	for (size_t i = 0; i < worker->displayData->robotNum; ++i) {
		std::string key = std::to_string(i);

		std::string teachPointStr = worker->displayData->teachPoints[i];
		// 当前选中机器人从面板保存
		if (i == worker->displayData->selectedRobot) {
			std::vector<int> rowList(mainWindow->ui->tableWidget->rowCount(), 0);
			for (size_t i = 0; i < rowList.size(); ++i)
				rowList[i] += i;
			save_teach_point(rowList, teachPointStr);
			worker->displayData->teachPoints[worker->displayData->selectedRobot] = teachPointStr;
		}

		if (teachPointStr.empty())
			continue;

		json["teach_point"][key] = nlohmann::json::parse(teachPointStr);
	}

	// 工艺参数保存 [procedure]
	nlohmann::json procedure;
	for (size_t i = 0; i < procedureWindow->procedure.size(); ++i) {
		auto proc = procedureWindow->procedure[i];

		// 焊接参数
		Arc_WeldingParaItem weldCfg = FSAIRobotInterface::deserialize_Arc_WeldingParaItem(proc);
		auto weldPair = FSAIRobotInterface::serialize_Arc_WeldingParaItem(weldCfg);
		procedure[std::to_string(i)][std::to_string(weldPair.first)] = weldPair.second;

		// 摆焊参数
		Weave waveCfg = FSAIRobotInterface::deserialize_Weave(proc);
		auto wavePair = FSAIRobotInterface::serialize_Weave(waveCfg);
		procedure[std::to_string(i)][std::to_string(wavePair.first)] = wavePair.second;

		// 跟踪参数
		Track trackCfg = FSAIRobotInterface::deserialize_Track(proc);
		auto trackPair = FSAIRobotInterface::serialize_Track(trackCfg);
		procedure[std::to_string(i)][std::to_string(trackPair.first)] = trackPair.second;

		// 运动参数
		FSAIRobotInterface::Move_Config moveCfg = FSAIRobotInterface::deserialize_Move_Config(proc);
		auto movePair = FSAIRobotInterface::serialize_Move_Config(moveCfg);
		procedure[std::to_string(i)][std::to_string(movePair.first)] = movePair.second;

	}
	json["procedure"] = procedure;

	// 写入文件
	//std::cout << std::setw(4) << json << std::endl;
	ofile << json << "\n";
	
	return 0;
}

// Ref: https ://leetcode.cn/problems/validate-ip-address/solutions/1521467/yan-zheng-ipdi-zhi-by-leetcode-solution-kge5/
// @return 0 - 非法IP地址; 1 - IPv4; 2 - IPv6
int validIPAddress(std::string queryIP) {
	if (queryIP.find('.') != std::string::npos) {
		// IPv4
		int last = -1;
		for (int i = 0; i < 4; ++i) {
			int cur = (i == 3 ? queryIP.size() : queryIP.find('.', last + 1));
			if (cur == std::string::npos) {
				return 0;
			}
			if (cur - last - 1 < 1 || cur - last - 1 > 3) {
				return 0;
			}
			int addr = 0;
			for (int j = last + 1; j < cur; ++j) {
				if (!isdigit(queryIP[j])) {
					return 0;
				}
				addr = addr * 10 + (queryIP[j] - '0');
			}
			if (addr > 255) {
				return 0;
			}
			if (addr > 0 && queryIP[last + 1] == '0') {
				return 0;
			}
			if (addr == 0 && cur - last - 1 > 1) {
				return 0;
			}
			last = cur;
		}
		return 1;
	}
	else {
		// IPv6
		int last = -1;
		for (int i = 0; i < 8; ++i) {
			int cur = (i == 7 ? queryIP.size() : queryIP.find(':', last + 1));
			if (cur == std::string::npos) {
				return 0;
			}
			if (cur - last - 1 < 1 || cur - last - 1 > 4) {
				return 0;
			}
			for (int j = last + 1; j < cur; ++j) {
				if (!isdigit(queryIP[j]) && !('a' <= tolower(queryIP[j]) && tolower(queryIP[j]) <= 'f')) {
					return 0;
				}
			}
			last = cur;
		}
		return 2;
	}
}
