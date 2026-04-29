#include "fsai_advanceConfigWindow.h"


AdvanceConfigWindow::AdvanceConfigWindow() {
	ui = std::make_shared<Ui_AdvanceConfigWindow>();

	ui->setupUi(this);
	
	// 关闭窗口
	QObject::connect(ui->pushButton_4, &QPushButton::released, this, [&]() {
		this->hide();
	});

	// 默认参数
	ui->comboBox->addItem("ZMotionRobot");
	ui->comboBox->addItem("ZVRRobot");
	ui->comboBox->addItem("FSAIRobot");
	ui->comboBox->addItem("FSAISimRobot");
	ui->comboBox->setEditable(false);

	ui->comboBox_2->addItem("1");
	ui->comboBox_2->addItem("2");
	ui->comboBox_2->addItem("3");
	ui->comboBox_2->addItem("4");
	ui->comboBox_2->setEditable(false);
}

AdvanceConfigWindow::~AdvanceConfigWindow() {

}


void AdvanceConfigWindow::refresh_display_data(const MainWindowDisplayData& data) {
	ui->comboBox->setCurrentIndex(data.interpAlgo);
}


void AdvanceConfigWindow::export_display_data(MainWindowDisplayData& data) {
	data.interpAlgo = ui->comboBox->currentIndex();
}
