#pragma once

#include <QDialog>

#include "ui_advanceConfigWindow.h"

#include "fsai_display_data.h"

class AdvanceConfigWindow : public QDialog {
public:
	// 主页面
	std::shared_ptr<Ui_AdvanceConfigWindow> ui;

private:


public:
	AdvanceConfigWindow();
	virtual ~AdvanceConfigWindow();

	// 输出当前显示数据
	void export_display_data(MainWindowDisplayData& data);

private:

	Q_OBJECT
public slots:
	// 刷新显示数据
	void refresh_display_data(const MainWindowDisplayData& data);

signals:
};
