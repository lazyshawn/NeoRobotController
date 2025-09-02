
#include "fsai_procedure.h"

ProcedureWindow::ProcedureWindow() {

	ui = std::make_shared<Ui_Procedure>();
	ui->setupUi(this);

	set_up_ui();

	// 关闭窗口
	QObject::connect(ui->buttonBox, &QDialogButtonBox::rejected, this, [&]() {
		this->hide();
	});

	// 新增工艺
	QObject::connect(ui->pushButton_3, &QPushButton::pressed, this, [&]() {
		int idx = ui->comboBox_4->count();
		procedure.push_back(std::map<int, std::vector<float>>());
		ui->comboBox_4->addItem("Proc. " + QString::number(idx));
		ui->comboBox_4->setCurrentIndex(idx);
	});

}

ProcedureWindow::~ProcedureWindow() {
}


void ProcedureWindow::set_up_ui() {

	// Ref: [子窗口位于父窗口上方，不阻塞父窗口](https://dev59.com/gY7ea4cB1Zd3GeqPBXbZ)
	//setWindowFlags(windowFlags() | Qt::Tool);

	// 工艺号
	procedure.clear();
	for (size_t i = 0; i < 10; ++i) {
		ui->comboBox_4->addItem("Proc. " + QString::number(i));
		procedure.push_back(std::map<int, std::vector<float>>());
	}

	// 焊接模式
	ui->comboBox->addItem("D.C.  ");
	ui->comboBox->addItem("Paulse");
	// 电压模式
	ui->comboBox_2->addItem("Unitary");
	ui->comboBox_2->addItem("Binary ");
	// 摆焊样式
	ui->comboBox_3->addItem("Sine     ");
	ui->comboBox_3->addItem("L-shape  ");
	ui->comboBox_3->addItem("Pendulum ");
	ui->comboBox_3->addItem("Triangle ");
	//ui->radioButton->setChecked(true);
}


