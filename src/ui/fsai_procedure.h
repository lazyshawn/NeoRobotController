#pragma once

#include <QDialog>
#include "ui_procedure.h"

#include <memory>

class ProcedureWindow : public QDialog {
public:
	// 主页面
	std::shared_ptr<Ui_Procedure> ui;

private:

	void set_up_ui();


public:
	ProcedureWindow();
	virtual ~ProcedureWindow();

	// 待修改的运动编号
	int cmdIdx = -1;

	// 工艺参数: 依赖主界面显示
	std::vector<std::map<int, std::vector<float>>> procedure;


private:

 Q_OBJECT
public slots:

};

