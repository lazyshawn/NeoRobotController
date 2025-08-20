#pragma once

#include <QDialog>
#include "ui_procedure.h"

#include <memory>

class ProcedureWindow : public QDialog {
public:
	// 主页面
	std::shared_ptr<Ui_Procedure> ui;

private:


public:
	ProcedureWindow();
	virtual ~ProcedureWindow();


private:

 Q_OBJECT
public slots:
	// void switch_online();
	// void jog_moving(int axis, int type);

};

