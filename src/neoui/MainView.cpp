#include "MainView.h"
#include <QDesktopWidget>
#include <QStyle>


MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_viewModel(new MainViewModel(this)) {

	setup_ui();

	bind_viewmodel();
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::showEvent(QShowEvent* event) {
	// 窗口居中显示
	this->setGeometry(
		QStyle::alignedRect(
			Qt::LeftToRight,
			Qt::AlignCenter,
			this->size(),
			QApplication::desktop()->availableGeometry()
		)
	);
}

void MainWindow::setup_ui() {
	// --- Qt 界面初始化
	ui->setupUi(this);

	// --- 连接地址
	ui->comboBox->addItem("");
	ui->comboBox->addItem("LOCAL");
	ui->comboBox->addItem("127.0.0.1");
	ui->comboBox->addItem("192.168.1.14");
	ui->comboBox->addItem("0");

	// --- 急停按钮，字体加粗
	ui->pushButton_34->setStyleSheet("font-weight: bold; border - radius: 10px;");

	// --- 机器人位置监控页面
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

	// --- 示教页面
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

void MainWindow::bind_viewmodel() {
	// --- 双向绑定: View 事件 → 调用 ViewModel 命令; ViewModel 属性变化 → 更新 View
	// 连接状态
	connect(ui->pushButton, &QPushButton::clicked, m_viewModel, &MainViewModel::change_connectState);
	connect(m_viewModel, &MainViewModel::connectState_changed, this, &MainWindow::on_connectState_changed);
	// 机械臂位置
	connect(m_viewModel, &MainViewModel::robot_pos_changed, this, &MainWindow::on_robot_pos_changed);
	// 速度
	connect(ui->spinBox, QOverload<int>::of(&QSpinBox::valueChanged), m_viewModel, &MainViewModel::change_speedRatio);
	connect(ui->horizontalSlider, &QSlider::sliderMoved, m_viewModel, &MainViewModel::change_speedRatio);
	connect(m_viewModel, &MainViewModel::speedRatio_changed, this, &MainWindow::on_speedRatio_changed);

	// --- 其他信号槽连接
	// 点动
	QPushButton *negJobBtn[9] = { ui->pushButton_12, ui->pushButton_14, ui->pushButton_16, ui->pushButton_18, ui->pushButton_20, ui->pushButton_22,
							      ui->pushButton_24, ui->pushButton_26, ui->pushButton_28 };
	QPushButton *posJobBtn[9] = { ui->pushButton_13, ui->pushButton_15, ui->pushButton_17, ui->pushButton_19, ui->pushButton_21, ui->pushButton_23,
								  ui->pushButton_25, ui->pushButton_27, ui->pushButton_29 };
	for (int i = 0; i < 9; ++i) {
		connect(negJobBtn[i], &QPushButton::pressed, m_viewModel, [&, i]() { m_viewModel->jog_move(0, i, -1, 1); });
		connect(negJobBtn[i], &QPushButton::released, m_viewModel, [&, i]() { m_viewModel->jog_move(0, i, -1, 0); });
		connect(posJobBtn[i], &QPushButton::pressed, m_viewModel, [&, i]() { m_viewModel->jog_move(0, i, 1, 1); });
		connect(posJobBtn[i], &QPushButton::released, m_viewModel, [&, i]() { m_viewModel->jog_move(0, i, 1, 0); });
	}
	// 选定机器人
	connect(ui->pushButton_30, &QPushButton::clicked, this, [&]() { robot_change(0); });
	connect(ui->pushButton_31, &QPushButton::clicked, this, [&]() { robot_change(1); });
	connect(ui->pushButton_32, &QPushButton::clicked, this, [&]() { robot_change(2); });
	connect(ui->pushButton_33, &QPushButton::clicked, this, [&]() { robot_change(3); });
	// 下发任务
	connect(ui->pushButton_4, &QPushButton::clicked, this, &MainWindow::set_auto_task);

	// --- 显示内容初始化
	on_connectState_changed();
	on_speedRatio_changed();
	robot_change(m_robotIdx);
}

void MainWindow::on_connectState_changed() {
	// 根据当前连接状态更新界面
	if (m_viewModel->cardConnectState() > 0) {
		ui->pushButton->setText("Disonnect");
	}
	else {
		ui->pushButton->setText("Connect");
	}
}

void MainWindow::on_robot_pos_changed() {
	// 更新机械臂位置
	auto jPos = m_viewModel->get_jPos();
	//ui->lineEdit->setText(QString::number(jPos[0]));
	for (size_t i = 0; i < 9; ++i) {
		float pos;
		// 关节位置
		pos = jPos.size() <= i ? 0 : jPos[i];
		QTableWidgetItem* seqItem = new QTableWidgetItem(QString::number(pos));
		ui->tableWidget_2->setItem(i, 1, seqItem);

		// 空间位置
		//pos = data.cPos.size() <= i ? 0 : data.cPos[i];
		//seqItem = new QTableWidgetItem(QString::number(pos));
		//ui->tableWidget_2->setItem(i, 2, seqItem);
	}
}

void MainWindow::on_speedRatio_changed() {
	// 获取当前速度
	double ratio = m_viewModel->speedRatio();
	// 更新界面
	ui->horizontalSlider->setValue(ratio);
	ui->spinBox->setValue(ratio);
}

void MainWindow::robot_change(int idx) {
	// 输入校验
	idx = idx < 0 ? 0 : idx;
	idx = idx > 3 ? 3 : idx;

	m_robotIdx = idx;
	QPushButton *switchRbtBtn[4] = { ui->pushButton_30, ui->pushButton_31, ui->pushButton_32, ui->pushButton_33 };
	for (int i = 0; i < 4; ++i) {
		switchRbtBtn[i]->setEnabled(i != idx);
	}
}

void MainWindow::set_auto_task() {
	// 轨迹
	FSAIRobotInterface::DiscreteTrajectory trajList;
	FSAIRobotInterface::SingleTrajectory curTraj;

	PosData dpos;
	dpos.pointType = 0;
	dpos.JntState = 0;
	dpos.rbtPos = std::vector<double>(6, 0.0);
	dpos.extPos = std::vector<double>(3, 0.0);

	// 点位数据
	PointInfo pointInfo;
	MotionCfg motionCfg;
	MoveCmd moveCmd;
	motionCfg.moveType = 1;
	motionCfg.speed = 2;
	motionCfg.smooth = 40;
	pointInfo.begPos = dpos;
	pointInfo.endPos = dpos;

	pointInfo.begPos = pointInfo.endPos;
	pointInfo.endPos.rbtPos[0] += 10;
	pointInfo.endPos.extPos[0] += 100;
	curTraj.set_data(pointInfo, motionCfg, moveCmd);
	trajList.add_single_traj(curTraj);

	pointInfo.begPos = pointInfo.endPos;
	pointInfo.endPos.rbtPos[1] += 10;
	pointInfo.endPos.extPos[0] += 100;
	curTraj.set_data(pointInfo, motionCfg, moveCmd);
	trajList.add_single_traj(curTraj);

	pointInfo.begPos = pointInfo.endPos;
	pointInfo.endPos.rbtPos[0] -= 10;
	pointInfo.endPos.extPos[0] += 100;
	curTraj.set_data(pointInfo, motionCfg, moveCmd);
	trajList.add_single_traj(curTraj);

	pointInfo.begPos = pointInfo.endPos;
	pointInfo.endPos.rbtPos[1] -= 10;
	pointInfo.endPos.extPos[0] += 100;
	curTraj.set_data(pointInfo, motionCfg, moveCmd);
	trajList.add_single_traj(curTraj);

	m_viewModel->set_auto_task(trajList);
}
