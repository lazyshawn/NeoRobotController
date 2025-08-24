
#include "fsai_mainwindow.h"

#include <memory>
#include<iostream>


MainWindow::MainWindow() {
	ui = std::make_shared<Ui_MainWindow>();

	ui->setupUi(this);

	robotButton = { ui->pushButton_30, ui->pushButton_31, ui->pushButton_32, ui->pushButton_33 };

	jogMoveBtn = { ui->pushButton_12, ui->pushButton_13, ui->pushButton_14, ui->pushButton_15,
		ui->pushButton_16, ui->pushButton_17, ui->pushButton_18, ui->pushButton_19,
		ui->pushButton_20, ui->pushButton_21, ui->pushButton_22, ui->pushButton_23,
		ui->pushButton_24, ui->pushButton_25, ui->pushButton_26, ui->pushButton_27,
		ui->pushButton_28, ui->pushButton_29 };
	jogMoveLabel = { ui->label, ui->label_2, ui->label_3, ui->label_4, ui->label_5, ui->label_6, ui->label_7, ui->label_8, ui->label_9 };
	jogTypeBtn = { ui->radioButton, ui->radioButton_2, ui->radioButton_3, ui->radioButton_4 };

	// UI 初始化
	set_up_ui();

	// 加载默认参数
	//displayData = std::shared_ptr<MainWindowDisplayData>(new MainWindowDisplayData);

	// 按默认参数显示
	update_display();

	// 加载槽函数
	connect_slot();
}

MainWindow::~MainWindow(){
}


/* ********************** 私有方法 ********************** */
void MainWindow::set_up_ui() {

	/* ********************** 控制页面 ********************** */
	ui->pushButton_34->setStyleSheet("font-weight: bold;border - radius: 10px;");

	/* ********************** 点动页面 ********************** */
	for (size_t i= 0; i < robotButton.size(); ++i) {
		robotButton[i]->setDisabled(i == 0);
	}

	/* ********************** 监控页面 ********************** */
	// 设置表头
	QStringList rowHeader;
	rowHeader << "Seq" << "Joint" << "TCP";
	ui->tableWidget_2->setColumnCount(rowHeader.count());
	ui->tableWidget_2->setHorizontalHeaderLabels(rowHeader);

	// 隐藏列表头
	ui->tableWidget_2->verticalHeader()->setVisible(false);
	// 交替色
	ui->tableWidget_2->setAlternatingRowColors(true);
	// 隐藏滑块
	ui->tableWidget_2->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	// 单击选中行
	ui->tableWidget_2->setSelectionBehavior(QAbstractItemView::SelectRows);

	for (size_t i = 0; i < 9; ++i) {
		ui->tableWidget_2->insertRow(i);
		// 序号
		QTableWidgetItem* seqItem = new QTableWidgetItem(QString::number(i));
		ui->tableWidget_2->setItem(i, 0, seqItem);
	}

	// Ref: https://stackoverflow.com/a/37615363
	ui->tableWidget_2->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
	ui->tableWidget_2->setMinimumHeight(ui->tableWidget_2->verticalHeader()->length() + ui->tableWidget_2->horizontalHeader()->height());

	ui->tableWidget_2->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	ui->tableWidget_2->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);


	/* ********************** 示教页面 ********************** */
	// 设置表头
	rowHeader.clear();
	rowHeader << "Seq" << "MType" << "Proc." << "JPos" << "CPos" << "External" << "Speed" << "Config";
	ui->tableWidget->setColumnCount(rowHeader.count());
	ui->tableWidget->setHorizontalHeaderLabels(rowHeader);

	// 隐藏列表头
	ui->tableWidget->verticalHeader()->setVisible(false);
	// 交替色
	ui->tableWidget->setAlternatingRowColors(true);
	// 单击选中行
	ui->tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);

	ui->tableWidget->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
	ui->tableWidget->setMinimumHeight(ui->tableWidget->verticalHeader()->length() + ui->tableWidget->horizontalHeader()->height());

	ui->tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	ui->tableWidget->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
	ui->tableWidget->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
	ui->tableWidget->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
	ui->tableWidget->horizontalHeader()->setSectionResizeMode(5, QHeaderView::ResizeToContents);
	ui->tableWidget->horizontalHeader()->setSectionResizeMode(6, QHeaderView::ResizeToContents);
	ui->tableWidget->horizontalHeader()->setSectionResizeMode(7, QHeaderView::ResizeToContents);

}

void MainWindow::connect_slot() {
	/* ********************** 控制页面 ********************** */
	// 选择机器人
	for (size_t i = 0; i < 4; ++i) {
		QObject::connect(robotButton[i], &QPushButton::pressed, this, [&, i]() {
			for (size_t j = 0; j < robotButton.size(); ++j) {
				robotButton[j]->setDisabled(j == i);
			}
			ui->textBrowser->append("Switch to robot " + QString::number(i));
		});
	}

	// 按键下发指令
	QObject::connect(ui->pushButton_8, &QPushButton::pressed, this, [&]() {
		auto cmd = ui->lineEdit->text();
		ui->textBrowser->append("> " + cmd);
	});
	// 回车下发指令
	QObject::connect(ui->lineEdit, &QLineEdit::returnPressed, this, [&]() {
		auto cmd = ui->lineEdit->text();
		ui->textBrowser->append("> " + cmd);
		ui->lineEdit->selectAll();
	});

	// 清空日志
	QObject::connect(ui->pushButton_9, &QPushButton::pressed, ui->textBrowser, &QTextBrowser::clear);

	// 点动
	for (int i = 0; i < 9; ++i) {
		QObject::connect(jogMoveBtn[2 * i], &QPushButton::pressed, this, [&, i]() {
			QString axisName = "[" + jogMoveLabel[i]->text() + "] ";
			ui->textBrowser->append(axisName + "-");
		});

		QObject::connect(jogMoveBtn[2 * i + 1], &QPushButton::pressed, this, [&, i]() {
			QString axisName = "[" + jogMoveLabel[i]->text() + "] ";
			ui->textBrowser->append(axisName + "+");
		});
	}

	// 记录示教点
	QObject::connect(ui->pushButton_2, &QPushButton::pressed, this, &MainWindow::record_teach_point);
	QObject::connect(ui->pushButton_3, &QPushButton::pressed, this, &MainWindow::delete_teach_point);

	/* ********************** 点动页面 ********************** */
	for (size_t i = 0; i < jogTypeBtn.size(); ++i) {
		QObject::connect(jogTypeBtn[i], &QRadioButton::clicked, this, [&,i]() {
			ui->textBrowser->append("Set jog mode: " + jogTypeBtn[i]->text());
		});
	}

}


/* ********************** 槽函数 ********************** */
// 用当前保存数据刷新更新显示
void MainWindow::update_display() {
}

// 更新状态数据并刷新界面
void MainWindow::update_display(const MainWindowDisplayData& data) {

	// 当前选中机器人
	int idx = data.selectedRobot;

	// 手自动模式
	int autoMode = data.robotMode[idx] % 2;
	ui->checkBox_2->setChecked(autoMode);
	ui->checkBox_2->setText(autoMode ? "Auto  " : "Manual");

	// 机器人运行模式
	int curRunStatus = data.runStatus[0];
	QString runStatusLabel;
	// 在线
	if (curRunStatus % 2) {
		runStatusLabel += "Online";
		ui->pushButton_30->setStyleSheet("background-color: rgb(96,96,96)");
	}
	else {
		runStatusLabel += "Offline";
	}
	// 空闲
	if ((curRunStatus >> 1) % 2) {
		runStatusLabel += "\nIdle";
		ui->pushButton_30->setStyleSheet("");
	}
	// 运行中
	else if ((curRunStatus >> 2) % 2) {
		runStatusLabel += "\nRunning";
		ui->pushButton_30->setStyleSheet("background-color: rgb(0,255,0)");
	}
	// 警告
	if ((curRunStatus >> 3) % 2) {
		runStatusLabel += "\nWarnning";
		ui->pushButton_30->setStyleSheet("background-color: rgb(255,255,51)");
	}
	// 异常
	if ((curRunStatus >> 4) % 2) {
		runStatusLabel += "\nError";
		ui->pushButton_30->setStyleSheet("background-color: rgb(255,0,0)");
	}
	// 改用 mouseMoveEvent
	ui->pushButton_30->setToolTip(runStatusLabel);


	// 位置监控
	for (size_t i = 0; i < 9; ++i) {
		float pos;
		// 关节位置
		pos = data.jPos.size() <= i ? 0 : data.jPos[i];
		QTableWidgetItem* seqItem = new QTableWidgetItem(QString::number(pos));
		ui->tableWidget_2->setItem(i, 1, seqItem);

		// 空间位置
		pos = data.cPos.size() <= i ? 0 : data.cPos[i];
		seqItem = new QTableWidgetItem(QString::number(pos));
		ui->tableWidget_2->setItem(i, 2, seqItem);
	}

}


/* ********************** 内部槽函数 ********************** */
void MainWindow::record_teach_point() {
	int row = ui->tableWidget->currentRow();
	// 若未选中则在最后插入，选中则在下方插入
	row = row < 0 ? ui->tableWidget->rowCount() : row + 1;

	// 新增一行
	ui->tableWidget->insertRow(row);
	// 保存示教点参数
	moveCfg.insert(moveCfg.begin() + row, std::map<int, std::vector<float>>());

	// 序号
	QTableWidgetItem* seqItem = new QTableWidgetItem(QString::number(row));
	seqItem->setFlags(seqItem->flags() & (~Qt::ItemIsEditable));
	ui->tableWidget->setItem(row, 0, seqItem);
	// 轨迹类型
	QComboBox* typeComboBox = new QComboBox();
	typeComboBox->addItem("     J");
	typeComboBox->addItem("     L");
	typeComboBox->addItem("     C");
	//typeComboBox->setButtonSymbols(QSpinBox::NoButtons);
	ui->tableWidget->setCellWidget(row, 1, typeComboBox);

	// 如果上一条运动为圆弧中间点，则当前类型为圆弧终点
	if (row > 0) {
		QWidget* preItem = ui->tableWidget->cellWidget(row - 1, 1);
		if (((QComboBox*)preItem)->currentIndex() == 2) {
			typeComboBox->setCurrentIndex(-1);
		}
	}
	QObject::connect(typeComboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, [&](int idx) {
		int row = ui->tableWidget->currentRow();
		// 下一条运动类型为圆弧终点
		if (idx == 2 && row + 1 < ui->tableWidget->rowCount()) {
			QWidget* preItem = ui->tableWidget->cellWidget(row + 1, 1);
			((QComboBox*)preItem)->setCurrentIndex(-1);
		}
		// 如果上一条运动为圆弧中间点，则当前类型为圆弧终点
		if (row > 0) {
			QWidget* preItem = ui->tableWidget->cellWidget(row - 1, 1);
			if (((QComboBox*)preItem)->currentIndex() == 2) {
				QWidget* preItem = ui->tableWidget->cellWidget(row, 1);
				((QComboBox*)preItem)->setCurrentIndex(-1);
			}
		}
		// 没有下一条轨迹则无法设置为圆弧中间点
		if (row + 1 >= ui->tableWidget->rowCount()) {
			QWidget* curItem = ui->tableWidget->cellWidget(row, 1);
			if (((QComboBox*)curItem)->currentIndex() == 2) {
				ui->textBrowser->append("Record end point of arc trajectory first");
				((QComboBox*)curItem)->setCurrentIndex(0);
			}
		}
	});

	// 工艺号
	QSpinBox* procedureBox = new QSpinBox();
	procedureBox->setMaximum(10);
	procedureBox->setPrefix("Proc. ");
	procedureBox->setButtonSymbols(QSpinBox::NoButtons);
	procedureBox->setAlignment(Qt::AlignHCenter);
	ui->tableWidget->setCellWidget(row, 2, procedureBox);

	// 更新后续行的行号
	for (size_t i = row + 1; i < ui->tableWidget->rowCount(); ++i) {
		ui->tableWidget->setItem(i, 0, new QTableWidgetItem(QString::number(i)));
	}

	// 选中新插入的行
	ui->tableWidget->selectRow(row);
	// 将焦点设置在table上，可以 <C-A> 选中所有
	ui->tableWidget->setFocus();



	// 当前行
	//row = ui->tableWidget->currentRow();

	// 从监控界面读取位置
	std::vector<float> jPos(9, 0);
	std::vector<float> cPos(9, 0);
	for (size_t i = 0; i < 3; ++i) {
		bool ok;   // 读取成功标志

		QTableWidgetItem *item = ui->tableWidget_2->item(i, 1);
		jPos[i] = item->text().toFloat(&ok);
		item = ui->tableWidget_2->item(i + 3, 1);
		jPos[i + 3] = item->text().toFloat(&ok);

		item = ui->tableWidget_2->item(i, 2);
		cPos[i] = item->text().toFloat(&ok);
		item = ui->tableWidget_2->item(i + 3, 2);
		cPos[i + 3] = item->text().toFloat(&ok);

		item = ui->tableWidget_2->item(i + 6, 1);
		jPos[i + 6] = item->text().toFloat(&ok);
		cPos[i + 6] = item->text().toFloat(&ok);
	}

	// 读取当前空间位置
	QString posStr = QString::number(jPos[0]);
	for (size_t i = 1; i < 6; ++i) {
		posStr += ", " + QString::number(jPos[i]);
	}
	QTableWidgetItem* item = new QTableWidgetItem(posStr);
	ui->tableWidget->setItem(row, 3, item);

	// 关节位置
	posStr = QString::number(cPos[0]);
	for (size_t i = 1; i < 6; ++i) {
		posStr += ", " + QString::number(cPos[i]);
	}
	item = new QTableWidgetItem(posStr);
	ui->tableWidget->setItem(row, 4, item);

	// 附加轴位置
	posStr = QString::number(jPos[6]);
	for (size_t i = 7; i < jPos.size(); ++i) {
		posStr += ", " + QString::number(jPos[i]);
	}
	item = new QTableWidgetItem(posStr);
	ui->tableWidget->setItem(row, 5, item);

	// 默认速度
	item = new QTableWidgetItem(QString::number(10.0));
	ui->tableWidget->setItem(row, 6, item);

	// 高级设置
	QPushButton* trajCfgBox = new QPushButton();
	//trajCfgBox->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	//trajCfgBox->setText("...");
	ui->tableWidget->setCellWidget(row, 7, trajCfgBox);

}

void MainWindow::delete_teach_point() {
	// 被移除的行
	std::vector<int> rowIdx;

	// Ref: 删除所有选中行
	// https://developer.baidu.com/article/details/2827678
	// 获取选择模型
	QItemSelectionModel* selectionModel = ui->tableWidget->selectionModel();
	// 获取所有选中的行索引
	QModelIndexList selectedRows = selectionModel->selectedRows();
	for (auto & selected : selectedRows) {
		rowIdx.push_back(selected.row());
	}

	if (rowIdx.size() < 1) return;
	std::sort(rowIdx.begin(), rowIdx.end());

	// 从前往后
	for (size_t i = 0; i < rowIdx.size(); ++i) {
		// 删除行
		ui->tableWidget->removeRow(rowIdx[i] - i);
		// 删除记录参数
		moveCfg.erase(moveCfg.begin() + rowIdx[i] - i);
	}

	// 更新行索引
	for (size_t i = 0; i < rowIdx.size(); ++i) {
		int beg = rowIdx[i], end = (i < rowIdx.size() - 1) ? rowIdx[i + 1] : ui->tableWidget->rowCount() + i;
		for (size_t j = beg; j < end; ++j) {
			ui->tableWidget->setItem(j - i, 0, new QTableWidgetItem(QString::number(j - i)));
		}
	}

	// 选中被删除的最后一行
	if (ui->tableWidget->rowCount() > 0) {
		size_t selected = rowIdx.back() - rowIdx.size() + 1;
		selected = selected >= ui->tableWidget->rowCount() ? ui->tableWidget->rowCount() - 1 : selected;
		ui->tableWidget->selectRow(selected);
		// 将焦点设置在table上，可以 <C-A> 选中所有
		ui->tableWidget->setFocus();
	}
}
