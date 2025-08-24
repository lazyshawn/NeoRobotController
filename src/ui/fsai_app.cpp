
#include "fsai_app.h"

#include <random>
#include <chrono>
#include <iostream>

#include "robot_interface/ZMotionRobot.h"
#include "robot_interface/ZRVRobot.h"
#include "robot_interface/FSAIRobot.h"

FSAIRobotInterface::RobotGroupManager group;

Worker::Worker() {
	// 主窗口显示数据
	displayData = std::shared_ptr<MainWindowDisplayData>(new MainWindowDisplayData);
	displayData->trajectory = std::vector<std::vector<std::vector<float>>>(displayData->robotNum);
}

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

	while (workerHealthy) {
		// 获取当前主界面状态
		//MainWindowDisplayData data;

		// 修改主界面状态
		FSAIRobotInterface::RobotStatus status;
		group.robotList[0]->get_rt_robot_status(status);
		displayData->jPos = status.jPos;
		displayData->cPos = status.cPos;

		// 手自动模式
		FSAIRobotInterface::set_bit(displayData->robotMode[0], 0, status.autoMode > 0);

		// 运行状态
		displayData->runStatus[0] = 0;
		// 在线/离线
		if (!FSAIRobotInterface::get_bit(status.lowerStatus, 6)) {
			FSAIRobotInterface::set_bit(displayData->runStatus[0], 0, true);
		}
		// 异常
		if (status.lowerStatus >> 2) {
			FSAIRobotInterface::set_bit(displayData->runStatus[0], 4, true);
		}
		// 暂停 / 警告
		if (FSAIRobotInterface::get_bit(status.lowerStatus, 1)) {
			FSAIRobotInterface::set_bit(displayData->runStatus[0], 3, true);
		}
		// 空闲 / 运行中
		if (FSAIRobotInterface::get_bit(status.lowerStatus, 0)) {
			FSAIRobotInterface::set_bit(displayData->runStatus[0], 2, true);
		}
		else {
			FSAIRobotInterface::set_bit(displayData->runStatus[0], 1, true);
		}


		// 暂停状态

		// 触发线程刷新
		emit update_data(*displayData);

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

	// 线程
	worker = new Worker;

	// 加载配置

	// 状态数据初始化
	//robotStatus = std::shared_ptr<RobotStatus>(new RobotStatus);

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
		// 设置可切换工艺
		procedureWindow->ui->comboBox_4->setDisabled(false);

		procedureWindow->cmdIdx = -1;

		// 更新工艺参数界面
		display_procedure_data(-1, -1);

		// 传入参数
		procedureWindow->show();
		procedureWindow->activateWindow();
	});

	// 设置窗口
	QObject::connect(advanceWindow->ui->buttonBox, &QDialogButtonBox::accepted, this, [&]() {
		// 插补算法类型
		int curInterpAlgo = advanceWindow->ui->comboBox->currentIndex();
		if (worker->displayData->interpAlgo != curInterpAlgo) {
			if (curInterpAlgo == 0) {
				robot = std::shared_ptr<FSAIRobotInterface::RobotBase>(new FSAIRobotInterface::ZMotionRobot);
			}
			else if (curInterpAlgo == 1) {
				robot = std::shared_ptr<FSAIRobotInterface::RobotBase>(new FSAIRobotInterface::ZRVRobot);
			}
			else if (curInterpAlgo == 2) {
				robot = std::shared_ptr<FSAIRobotInterface::RobotBase>(new FSAIRobotInterface::FSAIRobot);
			}
			mainWindow->ui->textBrowser->append("Switch InterpAlgo: " + advanceWindow->ui->comboBox->currentText());
		}

		advanceWindow->export_display_data(*(worker->displayData));

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
	// 连接机器人
	QObject::connect(mainWindow->ui->pushButton, &QPushButton::released, this, [&]() {
		//ZController->disconnect();
		//std::this_thread::sleep_for(std::chrono::milliseconds(10));

		int ret = ZController->lazy_connect();

		if (ret) {
			mainWindow->ui->textBrowser->append("Connect to controller failed");
			return;
		}
		else {
			mainWindow->ui->textBrowser->append("Connect to controller successful");
		}

		group.robotList[0]->set_ZController(ZController, group.robotList[0]->get_robotId());
	});

	// 切换手自动模式
	QObject::connect(mainWindow->ui->checkBox_2, &QCheckBox::released, this, [&]() {
		int idx = worker->displayData->selectedRobot;
		int goalMode = 1 - FSAIRobotInterface::get_bit(worker->displayData->robotMode[idx], 0);
		group.robotList[idx]->switch_auto(goalMode);
	});

	// 切换选中机器人
	for (size_t i = 0; i < 4; ++i) {
		QObject::connect(mainWindow->robotButton[i], &QPushButton::pressed, this, [&, i]() {
			worker->switch_robot(i);
		});
	}

	// 清空任务/报警
	QObject::connect(mainWindow->ui->pushButton_10, &QPushButton::pressed, this, [&]() {
		// 清除任务
		group.robotList[worker->displayData->selectedRobot]->task_stop();
		// 使能
		group.robotList[worker->displayData->selectedRobot]->switch_enable(true);
	});

	// 重启机器人
	QObject::connect(mainWindow->ui->pushButton_11, &QPushButton::pressed, this, [&]() {
		if (worker->displayData->interpAlgo == 0) {
			group.robotList[worker->displayData->selectedRobot]->reboot("./ctr/ZMotionRobot.zar");
		}
		else if (worker->displayData->interpAlgo == 1) {
			group.robotList[worker->displayData->selectedRobot]->reboot("./ctr/ZRVRobot.zar");
		}
		else if (worker->displayData->interpAlgo == 2) {
			group.robotList[worker->displayData->selectedRobot]->reboot("./ctr/FSAIRobot.zar");
		}
	});

	// 系统急停
	QObject::connect(mainWindow->ui->pushButton_34, &QPushButton::pressed, this, [&]() {
		for (auto& robot : group.robotList) {
			robot->emergency_stop();
		}
	});


	/* ********************** 示教页面 ********************** */
	// 开始
	QObject::connect(mainWindow->ui->pushButton_4, &QPushButton::pressed, this, [&]() {

		for (size_t i = 0; i < mainWindow->ui->tableWidget->rowCount(); ++i) {

			execute_teached_trajectory(i);

		}

	});
	// 执行当前
	QObject::connect(mainWindow->ui->pushButton_7, &QPushButton::pressed, this, [&]() {

		// 当前选中示教点
		int row = mainWindow->ui->tableWidget->currentRow();

		// 当前选中点为圆弧终点
		QWidget* widget = mainWindow->ui->tableWidget->cellWidget(row, 1);
		int moveType = ((QComboBox*)widget)->currentIndex();
		if (moveType < 0) {
			mainWindow->ui->textBrowser->append("Please selecte mid point of arc trajectory");
			return;
		}

		execute_teached_trajectory(row);

	});
	// 暂停
	QObject::connect(mainWindow->ui->pushButton_5, &QPushButton::pressed, this, [&]() {
		group.robotList[worker->displayData->selectedRobot]->task_pause();
		mainWindow->ui->pushButton_5->setText("Resume");
		//group.robotList[mainWindow->displayData->selectedRobot]->task_resume();
		//mainWindow->ui->pushButton_5->setText("Pause");
	});
	// 停止
	QObject::connect(mainWindow->ui->pushButton_6, &QPushButton::pressed, this, [&]() {
		group.robotList[worker->displayData->selectedRobot]->emergency_stop();
	});

	// 记录示教点
	QObject::connect(mainWindow->ui->pushButton_2, &QPushButton::pressed, this, [&]() {
		// 当前行
		int row = mainWindow->ui->tableWidget->currentRow();

		// 设置按钮
		QPushButton *btn = (QPushButton *)(mainWindow->ui->tableWidget->cellWidget(row, 7));

		// 绑定打开工艺参数
		connect(btn, &QPushButton::released, this, [&]() {
			// 设置不可切换工艺
			procedureWindow->ui->comboBox_4->setDisabled(true);

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


void FSAIApp::display_procedure_data(int procIdx, int cmdIdx) {

	// 未指定工艺号，按上次修改工艺显示
	if (procIdx < 0) {
		procIdx = procedureWindow->ui->comboBox_4->currentIndex();
	}

	// 未指定行号，仅修改工艺
	procedureWindow->ui->spinBox_11->setDisabled(procedureWindow->cmdIdx < 0);
	if (cmdIdx >= 0) {
		auto moveCfg = FSAIRobotInterface::deserialize_Move_Config(mainWindow->moveCfg[cmdIdx]);
		procedureWindow->ui->spinBox_11->setValue(moveCfg.smooth);
	}
	else {
		procedureWindow->ui->spinBox_11->setValue(-1);
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
	weldCfg.Id = procedureWindow->ui->groupBox->isChecked();                       // 0 起弧标志
	weldCfg.WeldingCrt_Spd = procedureWindow->ui->spinBox_12->value();             // 1 焊接电流
	weldCfg.WeldingVtg_Strth = procedureWindow->ui->spinBox->value();              // 2 焊接电压
	weldCfg.WeldingWorkMode = weldWorkMode;                                        // 3 焊接工作模式
	weldCfg.VtgUniCorrection = procedureWindow->ui->spinBox->value();              // 4 焊接电压修正值

	// 起弧参数 5
	weldCfg.ArcOnWorkMode = weldWorkMode;                                          // 0 起弧模式
	weldCfg.ArcOnCrt_Spd = procedureWindow->ui->spinBox_13->value();	       // 1 起弧电流
	weldCfg.ArcOnVtg_Strth = procedureWindow->ui->spinBox_14->value();	       // 2 起弧电压
	weldCfg.ArcOnTime = procedureWindow->ui->spinBox_2->value();	               // 3 起弧时间
	weldCfg.ArcOnVtg_Correction = procedureWindow->ui->spinBox_14->value();    // 4 起弧电压修正值
	//weldCfg.ArcOnBlowTime = param[num++];	 // 5 引气时间						   

	// 收弧参数 11
	weldCfg.ArcOffWorkMode = weldWorkMode;                                         // 0 收弧模式
	weldCfg.ArcOffCrt_Spd = procedureWindow->ui->spinBox_15->value();         // 1 收弧电流
	weldCfg.ArcOffVtg_Strth = procedureWindow->ui->spinBox_16->value();       // 2 收弧电压
	weldCfg.ArcOffTime = procedureWindow->ui->spinBox_3->value();             // 3 收弧时间
	weldCfg.ArcOffVtg_Correction = procedureWindow->ui->spinBox_16->value();  // 4 收弧电压修正值
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


	// 保存运动参数
	if (procedureWindow->cmdIdx >= 0) {
		FSAIRobotInterface::Move_Config moveCfg;
		moveCfg.smooth = procedureWindow->ui->spinBox_11->value();

		auto pair = FSAIRobotInterface::serialize_Move_Config(moveCfg);
		mainWindow->moveCfg[procedureWindow->cmdIdx][pair.first] = pair.second;
	}

}


void FSAIApp::execute_teached_trajectory(int row) {

	if (row < 0) {
		mainWindow->ui->textBrowser->append("Teach point not selected.");
		return;
	}

	// 当前行转换为轨迹
	DiscreteTrajectory traj;
	TrajectoryConfig trajCfg;

	// 运动类型
	QWidget* widget = mainWindow->ui->tableWidget->cellWidget(row, 1);
	int moveType = ((QComboBox*)widget)->currentIndex();

	// 工艺号
	widget = mainWindow->ui->tableWidget->cellWidget(row, 2);
	int procIdx = ((QSpinBox*)widget)->value();

	auto moveCfg = FSAIRobotInterface::deserialize_Move_Config(mainWindow->moveCfg[row]);
	// 速度
	float speed = mainWindow->ui->tableWidget->item(row, 6)->text().toFloat();
	// 平滑度
	trajCfg.smooth = moveCfg.smooth;

	// 关节运动
	if (moveType == 0) {
		// 获取点位
		std::vector<float> dpos = read_list_from_tableWidget(mainWindow->ui->tableWidget, row, { 3,5 });

		trajCfg.speed = speed > 100 ? 100 : speed;
		traj.moveJABS(dpos, trajCfg);
		int ret = group.robotList[0]->push_new_trajectory(traj);
		if (ret)
			mainWindow->ui->textBrowser->append("send traj error: " + QString::number(ret));
	}
	// 忽略圆弧运动终点
	else if (moveType < 0) {
		return;
	}
	// 空间运动
	else {
		// 获取点位
		std::vector<float> dpos = read_list_from_tableWidget(mainWindow->ui->tableWidget, row, { 4,5 });

		// 获取工艺参数
		for (auto& cfg : procedureWindow->procedure[procIdx]) {
			trajCfg.add_appendix(cfg);
		}
		// 非默认工艺，使用工艺指定的焊接速度
		trajCfg.speed = procIdx > 0 ? FSAIRobotInterface::deserialize_Move_Config(trajCfg.appendix).speed : speed;
		// 允许起弧
		Arc_WeldingParaItem weldCfg = FSAIRobotInterface::deserialize_Arc_WeldingParaItem(trajCfg.appendix);
		if (!mainWindow->ui->checkBox->isChecked())
			weldCfg.Id = 0;
		trajCfg.add_appendix(FSAIRobotInterface::serialize_Arc_WeldingParaItem(weldCfg));

		// 直线运动
		if (moveType == 1) {
			traj.moveLABS(dpos, trajCfg);
			int ret = group.robotList[0]->push_new_trajectory(traj);
			if (ret)
				mainWindow->ui->textBrowser->append("send traj error: " + QString::number(ret));
		}
		// 圆弧运动
		else {
			// 没有下一个点
			if (row + 1 >= mainWindow->ui->tableWidget->rowCount()) {
				mainWindow->ui->textBrowser->append("No end point of arc trajectory: " + QString::number(row));
				return;
			}

			// 下一个点不是圆弧终点
			QWidget* curItem = mainWindow->ui->tableWidget->cellWidget(row+1, 1);
			if (((QComboBox*)curItem)->currentIndex() >= 0) {
				mainWindow->ui->textBrowser->append("No end point of arc trajectory: " + QString::number(row));
				return;
			}

			// 获取终点位置
			std::vector<float> endPos = read_list_from_tableWidget(mainWindow->ui->tableWidget, row + 1, { 4,5 });

			traj.moveCABS(dpos, endPos, trajCfg);
			int ret = group.robotList[0]->push_new_trajectory(traj);
			if (ret)
				mainWindow->ui->textBrowser->append("send traj error: " + QString::number(ret));
		}
	}

}
