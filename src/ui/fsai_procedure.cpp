
#include "fsai_procedure.h"

ProcedureWindow::ProcedureWindow() : ui(new Ui_Procedure) {
	ui->setupUi(this);

	// 关闭窗口
	QObject::connect(ui->buttonBox, &QDialogButtonBox::rejected, this, [&]() {
		this->hide();
	});

	// Ref: https://cloud.tencent.com/developer/article/2372869
	// 默认关节运动
	ui->tabWidget->setTabText(0, "Proc. 0");
	// 默认空间运动
	ui->tabWidget->setTabText(1, "Proc. 1");

	// 新增工艺
	QObject::connect(ui->pushButton_3, &QPushButton::pressed, this, [&]() {
		int idx = ui->tabWidget->count();
		QWidget* tab = new QWidget;
		ui->tabWidget->addTab(tab, "Proc. " + QString::number(idx));
	});

	ui->comboBox->addItem("D.C.  ");
	ui->comboBox->addItem("Paulse");

	ui->comboBox_2->addItem("Unitary");
	ui->comboBox_2->addItem("Binary ");

	ui->comboBox_3->addItem("Sine     ");
	ui->comboBox_3->addItem("L-shape  ");
	ui->comboBox_3->addItem("Pendulum ");
	ui->comboBox_3->addItem("Triangle "); 
}

ProcedureWindow::~ProcedureWindow() {
}

