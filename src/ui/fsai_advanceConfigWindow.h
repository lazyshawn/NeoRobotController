#pragma once

#include <QDialog>

#include "ui_advanceConfigWindow.h"

class AdvanceConfigWindow : public QDialog {
public:
	// Ö÷Ò³Ãæ
	std::shared_ptr<Ui_AdvanceConfigWindow> ui;

private:


public:
	AdvanceConfigWindow();
	virtual ~AdvanceConfigWindow();


private:

	Q_OBJECT
public slots:
	// void switch_online();
	// void jog_moving(int axis, int type);

};
