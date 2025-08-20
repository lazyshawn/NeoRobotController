#include "fsai_advanceConfigWindow.h"

AdvanceConfigWindow::AdvanceConfigWindow() {
	ui = std::make_shared<Ui_AdvanceConfigWindow>();

	ui->setupUi(this);

	// 关闭窗口
	QObject::connect(ui->buttonBox, &QDialogButtonBox::rejected, this, [&]() {
		this->hide();
	});

	// 默认参数
	ui->comboBox->addItem("0, 1, 2, 3, 4, 5");
	ui->comboBox_2->addItem("6, 7, 8");
	ui->comboBox_3->addItem("9, 10, 11");
	ui->comboBox_3->addItem("15, 16, 17");
	ui->comboBox_4->addItem("12, 13, 14");
	ui->comboBox_5->addItem("36, 37, 38");
	ui->comboBox_6->addItem("40, 41, 42");
	ui->comboBox_7->addItem("18, 19, 20, 21, 22, 23, 24, 25, 26");
}

AdvanceConfigWindow::~AdvanceConfigWindow() {

}