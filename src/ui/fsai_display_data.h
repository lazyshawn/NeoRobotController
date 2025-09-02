#pragma once

// 主窗口显示数据
struct MainWindowDisplayData {

	// 全局配置
	// 机器人数量
	int robotNum = 4;
	// 算法类型
	int interpAlgo = 0;

	// 当前选中机器人
	int selectedRobot = 0;
	// 当前机器人位置
	std::vector<float> jPos;
	std::vector<float> cPos;
	// 当前IO状态

	// 运行状态: <在线, 空闲, 运行, 警告, 异常>
	std::vector<int> runStatus = { 0,0,0,0 };
	// 机器人状态
	//std::vector<int> errorStatus = { 0,0,0,0 };
	// 机器人运行状态 <手动/自动>
	std::vector<int> robotMode = { 0,0,0,0 };

	// 示教轨迹<运动类型, 工艺号, 关节位置, TCP位置, 地轨位置, 速度>
	std::vector<std::vector<std::vector<float>>> trajectory;
	
};
Q_DECLARE_METATYPE(MainWindowDisplayData);
