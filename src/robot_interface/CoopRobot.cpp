
#include "robot_interface/CoopRobot.h"

#include "RobotLogger.h"

#include <windows.h>
#include <iostream>

namespace FSAIRobotInterface {

// 机器人日志
log4cplus::Logger RobotLog::logger;
RobotLog::RobotLog() {
	log4cplus::helpers::SharedObjectPtr<log4cplus::Appender> _append;
	_append = log4cplus::helpers::SharedObjectPtr<log4cplus::Appender>(new log4cplus::RollingFileAppender("./log/ZMotionRobot.log", 8 * 1024 * 1024, 8));//按照固定大小进行log分割
	_append->setLayout(std::auto_ptr<log4cplus::Layout>(new log4cplus::PatternLayout(LOG4CPLUS_TEXT("%D{%m/%d/%Y %H:%M:%S:%q} [%t] %-5p - %m %n"))));//("%D{%m/%d/%y %H:%M:%S},大写的D代表北京时间否则不准																															/* step 4: Instantiate a logger object */
	logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("ZROBOT_LOG"));
	logger.setLogLevel(log4cplus::INFO_LOG_LEVEL);
	logger.addAppender(_append);

	LOG4CPLUS_INFO(logger, "*************************************\n"
		<< "RobotGroupManager Info:\n"
		<< "Version:         0.3.0.3\n"
		<< "Release Date:    250917");
}


Eigen::Matrix3f RobotConfig::get_slave_calibratino_mat() {

	auto rotEuler = std::vector<float>(slaveCalibration.begin() + 3, slaveCalibration.begin() + 6);

	return Eigen::AngleAxisf(rotEuler[2] * DT_PI / 180, Eigen::Vector3f::UnitZ()) *
		Eigen::AngleAxisf(rotEuler[1] * DT_PI / 180, Eigen::Vector3f::UnitY()) *
		Eigen::AngleAxisf(rotEuler[0] * DT_PI / 180, Eigen::Vector3f::UnitX()).matrix();

}


/* *************************** RobotBase *************************** */
RobotBase::~RobotBase() {
	return;
}

int RobotBase::wait_auto_task_stop() {
	std::unique_lock<std::mutex> lock(mtx);

	cvMotion.wait(lock, [&]() { return motionDone; });

	// 等待条件置反
	motionDone = false;

	if (robotStatus.lowerStatus == 0 && trajectory.trajectory_loaded() && robotStatus.lineNum == get_lineNum()) {
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " wake up and task list empty.");
	}
	else {
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " wake up, status: " <<
			robotStatus.lowerStatus << ", " << robotStatus.upperStatus << ", " << robotStatus.lineNum);
	}

	return 0;
}

int RobotBase::notify_waiting_robot() {

	// 防止虚假唤醒
	std::lock_guard<std::mutex> lock(mtx);
	motionDone = true;

	// 清空轨迹
	//clear_trajectory();

	// 唤醒线程
	cvMotion.notify_one();

	return 0;
}

int RobotBase::set_ZController(std::shared_ptr<Controller> ZController_, int id) {
	ZController = ZController_;

	// 机器人ID未指定
	if (id < 0) {
		// 获取可用的 robotId 号
		robotId = ZController->allocate_robot_id();

		// 使用分配的 robotId
		if (ZController->add_robot(robotId) != 0) {
			return -1;
		}
	}
	else {
		robotId = id;
	}

	// 从控制卡读取现有的机器人配置
	read_register_config();

	// 重置轨迹序号
	reset_line_num();

	// 读取机械臂当前状态
	update_rt_robot_status();

	return 0;
}

std::vector<int> RobotBase::get_joint_axis() {
	int base = robotId * 32;
	std::vector<int> axis = { base + 0,base + 1,base + 2,base + 3,base + 4,base + 5 };
	return axis;
}

std::vector<int> RobotBase::get_axis_idx() {
	int base = robotId * 32;
	bool coupling = static_cast<int>(robotConfig.couplingConfig[0]);

	std::vector<int> axis = { base + 0,base + 1,base + 2,base + 3,base + 4,base + 5 };
	if (coupling == 1) {
		axis[4] = base + 30;
		axis[5] = base + 31;
	}

	return axis;
}

std::vector<int> RobotBase::get_composed_axis(const std::vector<std::vector<int>>& axisList) {

	std::vector<int> ans;
	for (auto axis : axisList) {
		ans.insert(ans.end(), axis.begin(), axis.end());
	}
	return ans;

}

int RobotBase::get_rt_robot_status(RobotStatus& status) {

	{
		// 加锁
		std::lock_guard<std::mutex> lock(mtx);

		status = robotStatus;
	}


	return 0;
}

int RobotBase::set_upperStatus(int code) {
	// 加锁
	std::lock_guard<std::mutex> lock(mtx);

	robotStatus.upperStatus |= code;

	return 0;
}

int RobotBase::reset_upperStatus(int idx) {
	
	// 上位机状态位全部复位
	if (idx < 0) {
		{
			std::lock_guard<std::mutex> lock(mtx);
			robotStatus.upperStatus = 0;
		}

		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " reset upper status.");
	}
	// 指定Bit位复位
	else {

	}

	return 0;
}

int RobotBase::get_register_config(RobotConfig& config) {

	config = robotConfig;

	return 0;

}

int RobotBase::read_register_config() {
	int ret = 0, cfgNum = 0, cfgIdx = 0;
	//int cfgIdxBase = get_config_idx_base();
	std::vector<int> configIdx;

	// 读取配置
	std::vector<float> readValue;
	ZController->get_register(get_config_idx_base(), 300, readValue, 1);

	// 连杆长度: 从序号2开始，读取12个配置参数
	cfgIdx = 2;
	cfgNum = 12;
	robotConfig.linkLength.resize(cfgNum);
	for (size_t i = 0; i < cfgNum; ++i) {
		robotConfig.linkLength[i] = readValue[cfgIdx + i];
	}

	// 附加轴编号
	cfgNum = 3;
	// 下发
	cfgIdx = 205;
	robotConfig.appAxisIdx.resize(cfgNum);
	for (size_t i = 0; i < cfgNum; ++i) {
		robotConfig.appAxisIdx[i] = readValue[cfgIdx + i];
	}
	// 读取
	cfgIdx = 208;
	robotConfig.appAxisIdxRead.resize(cfgNum);
	for (size_t i = 0; i < cfgNum; ++i) {
		robotConfig.appAxisIdxRead[i] = readValue[cfgIdx + i];
	}

	// 编码器位数
	cfgIdx = 20;
	cfgNum = 9;
	robotConfig.encoderBit.resize(cfgNum);
	for (size_t i = 0; i < cfgNum; ++i) {
		robotConfig.encoderBit[i] = readValue[cfgIdx + i];
	}

	// 传动比
	cfgNum = 9;
	// 分子
	cfgIdx = 140;
	robotConfig.transRatioNumerator.resize(cfgNum);
	for (size_t i = 0; i < cfgNum; ++i) {
		robotConfig.transRatioNumerator[i] = readValue[cfgIdx + i];
	}
	//分母
	cfgIdx = 150;
	robotConfig.transRatioDenominator.resize(cfgNum);
	for (size_t i = 0; i < cfgNum; ++i) {
		robotConfig.transRatioDenominator[i] = readValue[cfgIdx + i];
	}

	// 耦合比
	cfgNum = 4;
	cfgIdx = 160;
	robotConfig.couplingConfig.resize(cfgNum);
	for (size_t i = 0; i < cfgNum; ++i) {
		robotConfig.couplingConfig[i] = readValue[cfgIdx + i];
	}

	// TCP
	cfgIdx = 111;
	cfgNum = 6;
	robotConfig.tcpPose.resize(cfgNum);
	for (size_t i = 0; i < cfgNum; ++i) {
		robotConfig.tcpPose[i] = readValue[cfgIdx + i];
	}

	// 关节上限位
	cfgIdx = 50;
	cfgNum = 9;
	robotConfig.jointSupremum.resize(cfgNum);
	for (size_t i = 0; i < cfgNum; ++i) {
		robotConfig.jointSupremum[i] = readValue[cfgIdx + i];
	}

	// 关节下限位
	cfgIdx = 40;
	cfgNum = 9;
	robotConfig.jointInfimum.resize(cfgNum);
	for (size_t i = 0; i < cfgNum; ++i) {
		robotConfig.jointInfimum[i] = readValue[cfgIdx + i];
	}

	// 最大关节速度(自动)
	cfgIdx = 60;
	cfgNum = 9;
	robotConfig.maxJointSpeedAuto.resize(cfgNum);
	for (size_t i = 0; i < cfgNum; ++i) {
		robotConfig.maxJointSpeedAuto[i] = readValue[cfgIdx + i];
	}

	// 最大关节速度(手动)
	cfgIdx = 70;
	cfgNum = 9;
	robotConfig.maxJointSpeedManual.resize(cfgNum);
	for (size_t i = 0; i < cfgNum; ++i) {
		robotConfig.maxJointSpeedManual[i] = readValue[cfgIdx + i];
	}

	// 最大末端速度(手动)
	cfgIdx = 85;
	cfgNum = 2;
	robotConfig.maxCartSpeedManual.resize(cfgNum);
	for (size_t i = 0; i < cfgNum; ++i) {
		robotConfig.maxCartSpeedManual[i] = readValue[cfgIdx + i];
	}

	// 附加轴标定结果
	cfgIdx = 91;
	cfgNum = 9;
	robotConfig.auxCalbration.resize(cfgNum);
	for (size_t i = 0; i < cfgNum; ++i) {
		robotConfig.auxCalbration[i] = readValue[cfgIdx + i];
	}

	// 零点编码器值
	cfgIdx = 100;
	cfgNum = 9;
	robotConfig.zeroEncoder.resize(cfgNum);
	for (size_t i = 0; i < cfgNum; ++i) {
		robotConfig.zeroEncoder[i] = readValue[cfgIdx + i];
	}

	// 主从机标定结果
	cfgIdx = 130;
	cfgNum = 9;
	robotConfig.slaveCalibration.resize(cfgNum);
	for (size_t i = 0; i < cfgNum; ++i) {
		robotConfig.slaveCalibration[i] = readValue[cfgIdx + i];
	}

	// IO 配置

	return ret;
}

int RobotBase::write_register_config(const RobotConfig& config) {

	int ret = 0, cfgNum = 0, cfgIdx = 0;
	int idxBase = get_config_idx_base();
	std::vector<int> configIdx;

	// 输入配置的合法性检查

	// 拷贝到当前程序
	robotConfig = config;

	// 配置缓存数组
	std::vector<float> readValue;

	// 连杆长度
	cfgIdx = 2;
	readValue = robotConfig.linkLength;
	ret = ZController->set_register(idxBase + cfgIdx, readValue, 1);

	// 附加轴编号
	// 下发
	cfgIdx = 205;
	for (size_t i = 0; i < robotConfig.appAxisIdx.size(); ++i) {
		readValue[i] = static_cast<float>(robotConfig.appAxisIdx[i]);
	}
	ret = ZController->set_register(idxBase + cfgIdx, readValue, 1);

	// 读取
	cfgIdx = 208;
	for (size_t i = 0; i < robotConfig.appAxisIdxRead.size(); ++i) {
		readValue[i] = static_cast<float>(robotConfig.appAxisIdxRead[i]);
	}
	ret = ZController->set_register(idxBase + cfgIdx, readValue, 1);

	// 编码器位数
	cfgIdx = 20;
	readValue = robotConfig.encoderBit;
	ret = ZController->set_register(idxBase + cfgIdx, readValue, 1);

	// 传动比
	// 分子
	cfgIdx = 140;
	readValue = robotConfig.transRatioNumerator;
	ret = ZController->set_register(idxBase + cfgIdx, readValue, 1);
	//分母
	cfgIdx = 150;
	readValue = robotConfig.transRatioDenominator;
	ret = ZController->set_register(idxBase + cfgIdx, readValue, 1);

	// 耦合比
	cfgIdx = 160;
	readValue = robotConfig.couplingConfig;
	ret = ZController->set_register(idxBase + cfgIdx, readValue, 1);

	// TCP
	cfgIdx = 111;
	readValue = robotConfig.tcpPose;
	ret = ZController->set_register(idxBase + cfgIdx, readValue, 1);


	// 关节上限位
	cfgIdx = 50;
	readValue = robotConfig.jointSupremum;
	ret = ZController->set_register(idxBase + cfgIdx, readValue, 1);

	// 关节下限位
	cfgIdx = 40;
	readValue = robotConfig.jointInfimum;
	ret = ZController->set_register(idxBase + cfgIdx, readValue, 1);

	// 最大关节速度(自动)
	cfgIdx = 60;
	readValue = robotConfig.maxJointSpeedAuto;
	ret = ZController->set_register(idxBase + cfgIdx, readValue, 1);

	// 最大关节速度(手动)
	cfgIdx = 70;
	readValue = robotConfig.maxJointSpeedManual;
	ret = ZController->set_register(idxBase + cfgIdx, readValue, 1);

	// 最大末端速度(手动)
	cfgIdx = 85;
	readValue = robotConfig.maxCartSpeedManual;
	ret = ZController->set_register(idxBase + cfgIdx, readValue, 1);

	// 附加轴标定结果
	cfgIdx = 91;
	readValue = robotConfig.auxCalbration;
	ret = ZController->set_register(idxBase + cfgIdx, readValue, 1);

	// 零点编码器值
	cfgIdx = 100;
	readValue = robotConfig.zeroEncoder;
	ret = ZController->set_register(idxBase + cfgIdx, readValue, 1);

	// 主从机标定结果
	cfgIdx = 130;
	readValue = robotConfig.slaveCalibration;
	ret = ZController->set_register(idxBase + cfgIdx, readValue, 1);

	// IO 配置

	//read_register_config();

	LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " write register config.");

	return ret;
}

int RobotBase::capture_controller_log() {
	ZController->read_message();
	return 0;
}

int RobotBase::reboot(const char *basPath, int mode) {
	if (mode < 0 || mode > 1)
		return -1;

	return ZController->load_basic_project(basPath, mode);
}

int RobotBase::load_config(const std::string& fname) {
	return ZController->load_config(fname);
}

int RobotBase::export_config(const std::string& fname) {
	return ZController->export_config(fname);
}

int RobotBase::set_axisIdxMask(const std::vector<int>& axis) {
	axisMask = std::unordered_set<int>(axis.begin(), axis.end());

	return 0;
}

int RobotBase::find_command_axis(const SingleTrajectory& traj, int axis) {
	auto pnt = traj.mainPoint;
	std::vector<int> axisList = robotConfig.appAxisIdx;

	int axisId = -1;
	for (size_t i = 0; i < axisList.size(); ++i) {
		if (axis == axisList[i]) {
			axisId = i;
			break;
		}
	}

	if (pnt.size() > axisId)
		return axisId;

	return -1;
}

int RobotBase::execute_move_action(const std::vector<std::pair<int, std::vector<float>>>& actionList, int flag) {
	
	int stateIdxBase = get_state_idx_base();
	std::vector<int> axis = get_execute_axis();

	for (const auto& action : actionList) {
		auto type = action.first;
		auto param = action.second;

		// 完成标志位复位
		ZController->set_axis_param(stateIdxBase + 350, "TABLE", 0, axis[0]);

		// 下发运动参数
		if (param.size() > 0) {
			std::vector<int> idx(param.size(), stateIdxBase + 301);
			for (size_t i = 0; i < idx.size(); ++i) {
				idx[i] += i;
			}
			ZController->set_axis_param(idx, "TABLE", param, axis[0]);
		}

		// 下发运动
		ZController->set_axis_param(stateIdxBase + 300, "TABLE", type, axis[0]);

		// 等待运动结束
		ZController->move_wait(axis[0], "TABLE", stateIdxBase + 350, 0, 1);

		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " Move_Action " << type << (param.size() > 0 ? ": " : "") << vector_to_string(param));
	}

	return 0;
}

int RobotBase::process_after_send_traj() {
	return 0;
}

int RobotBase::send_traj_type(int type) {
	return 0;
}

int RobotBase::trigger_action(int type, const std::vector<float>& param) {
	
	int stateIdxBase = get_state_idx_base();
	//std::vector<int> axis = get_execute_axis();

	// 下发运动参数
	if (param.size() > 0) {
		std::vector<int> idx(param.size(), stateIdxBase + 301);
		for (size_t i = 0; i < idx.size(); ++i) {
			idx[i] += i;
		}
		ZController->set_axis_param(idx, "TABLE", param);
	}

	// 完成标志位复位
	//ZController->set_axis_param(stateIdxBase + 350, "TABLE", 0, axis[0]);

	// 下发运动
	ZController->set_axis_param(stateIdxBase + 300, "TABLE", type);

	LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " trigger Move_Action "
		<< type << (param.size() > 0 ? ": " : "") << vector_to_string(param));

	return 0;
}


int RobotBase::export_tracking_data() {
	// 检测文件夹是否存在
	std::string foldName = "log/tracking_data";
	DWORD attribs = ::GetFileAttributesA(foldName.c_str());
	if (attribs == INVALID_FILE_ATTRIBUTES) {
		::CreateDirectoryA(foldName.c_str(), NULL);
	}
	// 创建子文件夹
	foldName = "log/tracking_data/" + std::to_string(aliasId);
	attribs = ::GetFileAttributesA(foldName.c_str());
	if (attribs == INVALID_FILE_ATTRIBUTES) {
		::CreateDirectoryA(foldName.c_str(), NULL);
	}

	// 保存原始电流数据
	auto idx = get_data_idx_base();
	auto fileName = foldName;
	fileName = foldName + "/raw_current.txt";
	ZController->save_table(idx + 10000, 5000, fileName);

	// 保持滤波电流数据
	fileName = foldName + "/fine_current.txt";
	ZController->save_table(idx + 15000, 5000, fileName);

	// 保存左右端点索引
	fileName = foldName + "/index.txt";
	ZController->save_table(idx + 20000, 5000, fileName);

	// 保存配置
	idx = get_state_idx_base();
	fileName = foldName + "/config.txt";
	ZController->save_table(idx + 180, 50, fileName);

	return 0;
}

int RobotBase::get_input_effective_state(const std::vector<int>& ioNum, std::vector<int>& state) {
	int num = (std::min)(ioNum.size(), state.size());

	std::vector<float> value = std::vector<float>(state.begin(), state.end());
	std::vector<int> vrIdx(num, get_config_idx_base() + 300);
	for (size_t i = 0; i < num; ++i) {
		vrIdx[i] += 2 * ioNum[i];
	}

	int ret = ZController->get_axis_param(vrIdx, "VR", value);
	state.clear();
	for (size_t i = 0; i < value.size(); ++i) {
		state.push_back(static_cast<int>(value[i]));
	}

	return 0;
}

int RobotBase::set_input_effective_state(const std::vector<int>& ioNum, const std::vector<int>& state) {

	int num = (std::min)(ioNum.size(), state.size());

	std::vector<float> value = std::vector<float>(state.begin(), state.end());
	std::vector<int> vrIdx(num, get_config_idx_base() + 300);
	for (size_t i = 0; i < num; ++i) {
		vrIdx[i] += 2 * ioNum[i];
	}

	int ret = ZController->set_axis_param(vrIdx, "VR", value);

	return 0;

}

int RobotBase::get_input(int ioNum) {
	return ZController->get_in(ioNum);
}

int RobotBase::get_input_invert(int ioNum) {
	return ZController->get_invert_in(ioNum);
}

int RobotBase::set_input_invert(int ioNum, int state) {

	if (state > 0) {
		// 反转关
		ZController->set_invert_in(ioNum, 1);
	}
	else if (state <= 0) {
		// 反转开
		ZController->set_invert_in(ioNum, 0);
	}
	return 0;
}


int RobotBase::get_output(int ioNum) {

	return ZController->get_op(ioNum);

}


int RobotBase::set_output(int ioNum, int state) {

	return ZController->set_op(ioNum, state);

}


int RobotBase::send_welding_wire(int dir) {

	int stateIdxBase = get_state_idx_base();
	int ret = 0;
	// 送丝
	if (dir > 0) {
		ZController->set_axis_param(stateIdxBase + 55, "TABLE", 1);
	}
	// 抽丝
	else if (dir < 0) {
		ZController->set_axis_param(stateIdxBase + 55, "TABLE", -1);
	}
	// 停止送丝/抽丝
	else {
		ZController->set_axis_param(stateIdxBase + 55, "TABLE", -2);
	}

	return ret;

}


int RobotBase::welder_blow(bool on) {

	int stateIdxBase = get_state_idx_base();
	int ret = 0;
	if (on) {
		ZController->set_axis_param(stateIdxBase + 55, "TABLE", 2);
	}
	else {
		ZController->set_axis_param(stateIdxBase + 55, "TABLE", -2);
	}
	return ret;

}

int RobotBase::send_command(const std::string& cmd, std::string& ack, int type) {
	char cmdbuffAck[2048];
	int ret = ZController->sendCmd(cmd.c_str(), cmdbuffAck, type);
	ack = cmdbuffAck;

	return ret;
}

int RobotBase::reset_dual_axis_home_position() {
	char cmdbuff[2048], tempbuff[2048], cmdbuffAck[2048];

	strcpy(cmdbuff, "RUNTASK 6, SET_DUAL_AXIS_ZERO");

	int ret = ZController->sendCmd(cmdbuff, cmdbuffAck);

	return ret;
}

int RobotBase::read_action_result(std::vector<float>& result) {

	int stateIdxBase = get_state_idx_base(), ret = 0;
	std::vector<float> value;
	std::vector<int> idx(49, stateIdxBase + 351);
	for (size_t i = 0; i < idx.size(); ++i) {
		idx[i] += i;
	}

	ZController->get_axis_param(idx, "TABLE", result);

	// 打印输出结果

	return ret;
}

int RobotBase::get_multilayer_pos(std::vector<float>& pos) {
	int dataIdxBase = get_data_idx_base();
	std::vector<float> data;

	// 轨迹段数
	ZController->get_axis_param({ dataIdxBase + 25000 }, "TABLE", data);
	int num = static_cast<int>(data[0]);

	LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << ": " << num << " multilayer data recorded.");
	if (num < 0)
		return -1;

	std::vector<int> idxList(10, dataIdxBase + 25000);
	for (size_t i = 0; i < idxList.size(); ++i)
		idxList[i] += i;
	ZController->get_axis_param(idxList, "TABLE", data);
	pos.clear();
	// 轨迹段数
	pos.push_back(data[0]);
	// 起点
	std::vector<float> tmp = std::vector<float>(data.begin() + 1, data.begin() + 10);
	cpos_base_to_world(tmp);
	pos.insert(pos.end(), tmp.begin(), tmp.end());

	for (size_t i = 0; i < num; ++i) {
		idxList = std::vector<int>(20, dataIdxBase + 25000 + 10 + 20 * i);
		for (size_t j = 0; j < idxList.size(); ++j)
			idxList[j] += j;
		ZController->get_axis_param(idxList, "TABLE", data);

		// 轨迹类型
		pos.insert(pos.end(), data[0]);
		// 轨迹编号
		pos.insert(pos.end(), data[1]);

		// 中间点
		std::vector<float> tmp = std::vector<float>(data.begin() + 2, data.begin() + 11);
		cpos_base_to_world(tmp);
		pos.insert(pos.end(), tmp.begin(), tmp.end());

		// 结束点
		tmp = std::vector<float>(data.begin() + 11, data.begin() + 20);
		cpos_base_to_world(tmp);
		pos.insert(pos.end(), tmp.begin(), tmp.end());
	}

	// 清空数据
	data = std::vector<float>(idxList.size(), 0.0);
	ZController->set_axis_param({ dataIdxBase + 25000 }, "TABLE", { 0.0 });

	return 0;
}

int RobotBase::get_slave_buffer() {
	
	int begIdx = get_data_idx_base();
	int ret = ZController->get_register(begIdx + 21000, 500, statusBuffer.slaveBuffer, 0);

	return 0;
}

int RobotBase::single_axis_enable(bool enable, int axis) {
	// 总开关
	if (axis = -1) {

	}

	// 单轴使能

	return 0;
}

/* *************************** RobotGroupManager *************************** */
RobotGroupManager::RobotGroupManager() {
	cmdThreadDone = true;
}

RobotGroupManager::~RobotGroupManager() {
	if (cmdThreadWorker.joinable()) {
		cmdThreadWorker.join();
	}
}


int RobotGroupManager::new_robot(std::shared_ptr<RobotBase> robot) {

	robotList.push_back(robot);
	// 设置别名ID
	robot->set_aliasId(robotList.size() - 1);

	syncState.push_back({});
	syncReadyState.push_back(0);
	waitState.push_back({});

	trajHistory.push_back({});
	coopState.push_back(0);
	disableGroup.push_back({});

	return 0;
}


int RobotGroupManager::set_shared_axis(int axisId, const std::vector<int>& robotId) {

	//// 查找共用轴情况
	//std::map<int, std::vector<int>>::iterator ite = sharedAxis.find(axisId);
	//// 修改共用轴的机器人ID
	//sharedAxis[axisId] = robotId;
	//// 公用轴状态
	//sharedAxisState[axisId] = -1;
	//return ite != sharedAxis.end();

	sharedAxisState.first = axisId;
	sharedAxisState.second = -1;

	return 0;

}


int RobotGroupManager::start_thread() {

	// 状态刷新线程使能
	for (auto& robot : robotList) {
		//robot->enable_refresh_thread(true);
	}

	// 绑定成员函数和 this 指针
	//cmdThreadWorker = std::thread(&RobotGroupManager::processCommandThread, this);
	updateThreadWorker = std::thread(&RobotGroupManager::updateStatusThread, this);

	LOG4CPLUS_INFO(RobotLog::getLogger(), "Process Command Thread Begin.");

	return 0;
}


int RobotGroupManager::stop() {

	workerHealthy = false;
	// 结束工作线程
	if (updateThreadWorker.joinable())
		updateThreadWorker.join();
	if (cmdThreadWorker.joinable())
		cmdThreadWorker.join();

	// 状态刷新线程使能
	for (auto& robot : robotList) {
		//robot->enable_refresh_thread(false);
		// 唤醒等待中的线程
		robot->notify_waiting_robot();
	}

	return 0;

}


void RobotGroupManager::processCommandThread() {
	cmdThreadDone.store(false);
	// 指令返回值
	int ret = 0;
	// 获取当前时间戳
	auto start = std::chrono::steady_clock::now();
	// 下次唤醒时间
	auto wakeUpTime = start;
	// 线程周期(ms)
	long long duration = 50;


	// 指令执行线程
	while (workerHealthy) {
		
		for (size_t i = 0; i < robotList.size(); ++i) {
			// 开始执行轨迹: 设定上条轨迹，计算协同轨迹时间
			robotList[i]->set_ready_for_consistent_traj();

			// 更新每台机器人当前的同步设置
			set_group_sync_config(i);
		}

		// 轨迹动作处理
		for (size_t i = 0; i < robotList.size(); ++i) {
			// 更新同步状态
			update_sync_state(i);
		}

		// Group 状态处理
		// 协同轨迹速度更新
		//correct_sync_speed();
		// 所有机器人空闲(协同完成)，取消地轨屏蔽
		if (robot_group_idle()) {
			// 打印取消屏蔽日志
			for (size_t i = 0; i < robotList.size(); ++i) {
				robotList[i]->set_axisIdxMask({});
			}
			if (sharedAxisState.second > 0) {
				LOG4CPLUS_INFO(RobotLog::getLogger(), "Unset mask of shared axis: " << sharedAxisState.first);
			}
			sharedAxisState.second = -1;
		}
		
		// 指令下发
		for (size_t i = 0; i < robotList.size(); ++i) {

			// 指令缓存不为空
			while (!robotList[i]->trajectory.trajectory_loaded()) {

				// 机器人状态警告,不清空轨迹: 处于暂停状态
				if (robot_warning(i))
					break;

				// 机器人出现错误
				if (robot_error(i))
					break;

				// 运动完成
				if (robot_idle(i))
					break;

				// 获取当前轨迹
				auto curTraj = robotList[i]->trajectory.get_curTraj();
				// 获取上一条轨迹
				auto preTraj = robotList[i]->trajectory.get_preTraj();

				// 轨迹一致性不满足
				if (!robotList[i]->consistent_traj_ready(coopState[i]))
					break;

				// 需要等待同步和协同: 空闲 + 同步号相同
				if (!robot_sync_ready(i)) {
					if (get_bit(coopState[i], 3) == 0) {
						LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << i << " not synced: "
							<< vector_to_string(serialize_Sync_Config(deserialize_Sync_Config(curTraj.appendix)).second));
						set_bit(coopState[i], 3, true);
					}
					break;
				}
				else {
					set_bit(coopState[i], 3, false);
				}

				// IO 同步标志复位
				//reset_wait_state(i);

				// 轨迹分割
				if (curTraj.isCartesian()) {
					// 拆分轨迹
					robotList[i]->separate_trajectory();
					// 更新轨迹
					curTraj = robotList[i]->trajectory.get_curTraj();
				}

				// 指令缓存检测
				if (!robotList[i]->remain_buffer_free()) {
					if (get_bit(coopState[i], 2) == 0) {
						set_bit(coopState[i], 2, true);
						LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << i << " buffer not enough: "
							<< "Move to: " << vector_to_string(curTraj.mainPoint));
					}
					break;
				}
				else {
					set_bit(coopState[i], 2, false);
				}

				// 检查地轨指令是否已经下发
				if (robotList[i]->find_command_axis(curTraj, sharedAxisState.first) >= 0 && sharedAxisState.second >= 0 && sharedAxisState.second != i) {
					// 屏蔽当前机器人的公用地轨轴
					robotList[i]->set_axisIdxMask({ sharedAxisState.first });
				}
				else {
					// 地轨轴指令跟随当前机器人发送
					sharedAxisState.second = i;
				}

				// 执行运动前动作
				auto action = deserialize_Move_Action(curTraj.appendix);
				robotList[i]->execute_move_action(action.actionBefore, 0);

				// 下发运动指令
				if (curTraj.isJoint()) {
					ret = robotList[i]->execute_single_joint();
				}
				else if (curTraj.isCartesian()){
					// 下发轨迹
					ret = robotList[i]->execute_single_cartesian();
				}

				// 下发异常处理
				if (ret != 0) {
					LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << i << " send command failed: " << ret);
					robotList[i]->set_upperStatus(0x20);
					break;
				}

				// 执行运动后动作
				robotList[i]->execute_move_action(action.actionAfter, 1);

				// 轨迹下发后的处理
				robotList[i]->process_after_send_traj();

				// 记录当前轨迹编号，用于轨迹完成后的触发动作
				curTraj.lineNum = robotList[i]->get_lineNum();
				trajHistory[i].push_back(curTraj);

			}

		}

		// 所有指令下发完毕
		bool taskFinish = true;
		for (size_t i = 0; i < robotList.size(); ++i) {
			if (!robotList[i]->trajectory.trajectory_loaded()) {
				taskFinish = false;
				break;
			}
		}
		if (taskFinish) {
			cmdThreadDone.store(true);
			return;
		}

		// 设置下次唤醒时间
		wakeUpTime += std::chrono::milliseconds(duration);
		// 休眠
		auto now = std::chrono::steady_clock::now();

		if ((now - wakeUpTime).count() > 0) {
			// 周期时间耗尽
			long long detTime = std::chrono::duration_cast<std::chrono::milliseconds>(now - wakeUpTime).count();
			wakeUpTime += std::chrono::milliseconds((detTime / duration + 1) * duration);
		}
		else {
			std::this_thread::sleep_until(wakeUpTime);
		}
	}


	LOG4CPLUS_INFO(RobotLog::getLogger(), "Process Command Thread Terminated.");

}


void RobotGroupManager::updateStatusThread() {
	// 指令返回值
	int ret = 0;
	// 获取当前时间戳
	auto start = std::chrono::steady_clock::now();
	// 下次唤醒时间
	auto wakeUpTime = start;
	// 线程周期(ms)
	long long duration = 50;
	// 保存机器人状态
	statusList.resize(robotList.size());

	// 指令执行线程
	while (workerHealthy) {

		// 更新状态
		for (size_t i = 0; i < robotList.size(); ++i) {
			// 更新机器人状态 (唯一更新途径)
			robotList[i]->update_rt_robot_status();
			// 获取机器人状态: 保证当前周期使用相同的机器人状态
			robotList[i]->get_rt_robot_status(statusList[i]);
		}

		// 获取下位机缓冲数据
		//for (size_t i = 0; i < robotList.size(); ++i) {
		//	robotList[i]->get_slave_buffer();
		//}

		// 获取下位机缓冲数据
		for (size_t i = 0; i < robotList.size(); ++i) {
			// 轨迹到位处理
			robot_in_place_command(i);

			// 捕获下位机日志
			robotList[i]->capture_controller_log();
		}

		// 指令缓存检测，指令下发线程未运行则开启
		for (size_t i = 0; i < robotList.size(); ++i) {

			// 可以下发任务
			bool startCmdThread = true;

			// 机器人状态警告,不清空轨迹: 处于暂停状态
			if (robot_warning(i)) {
				// 关联机器人同步进入暂停
				pause_coop_robot(i);
				startCmdThread = false;
				//continue;
			}

			// 机器人出现错误
			if (robot_error(i)) {
				robotList[i]->notify_waiting_robot();
				// 关联机器人同步进入暂停
				pause_coop_robot(i);
				startCmdThread = false;
				//continue;
			}

			// 运动完成，清空轨迹
			if (robot_idle(i)) {
				robotList[i]->notify_waiting_robot();
				startCmdThread = false;
				//continue;
			}

			// 机器人未异常，指令下发程序，轨迹未下发完成
			if (startCmdThread && cmdThreadDone && !robotList[i]->trajectory.trajectory_loaded()) {
				if (cmdThreadWorker.joinable())
					cmdThreadWorker.join();
				cmdThreadWorker = std::thread(&RobotGroupManager::processCommandThread, this);
				break;
			}
		}


		// 设置下次唤醒时间
		wakeUpTime += std::chrono::milliseconds(duration);
		// 休眠
		auto now = std::chrono::steady_clock::now();
		if ((now - wakeUpTime).count() > 0) {
			// 周期时间耗尽
			long long detTime = std::chrono::duration_cast<std::chrono::milliseconds>(now - wakeUpTime).count();
			wakeUpTime += std::chrono::milliseconds((detTime / duration + 1) * duration);
			LOG4CPLUS_INFO(RobotLog::getLogger(), "Cycle time exausted." << detTime);
		}
		else
			std::this_thread::sleep_until(wakeUpTime);

	}

}


bool RobotGroupManager::robot_error(int idx) {

	// 下位机无异常，上位机无异常
	if ((statusList[idx].lowerStatus >> 2) == 0 && statusList[idx].upperStatus == 0) {
		set_bit(coopState[idx], 1, false);
		return false;
	}
    else if (get_bit(coopState[idx], 1) == 0) {
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << idx << " error occur: "
			<< statusList[idx].lowerStatus << ", " << statusList[idx].upperStatus << ", " << coopState[idx] << ".\n"
			<< "Pos: " << vector_to_string(statusList[idx].cPos));

		set_bit(coopState[idx], 1, true);
	}

	return true;

}

bool RobotGroupManager::robot_warning(int idx) {

	if (get_bit(statusList[idx].lowerStatus, 1) == 0) {
		set_bit(coopState[idx], 0, false);
	}
	// 机器人处于暂停状态
	else {
		if (get_bit(coopState[idx], 0) == 0) {
			LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << idx << " warning occur: "
				<< statusList[idx].lowerStatus << ", " << statusList[idx].upperStatus << ", " << coopState[idx] << ".\n"
				<< "Pos: " << vector_to_string(statusList[idx].cPos));

			set_bit(coopState[idx], 0, true);
		}
		return true;
	}

	// 上位机未触发暂停
	//if (get_bit(coopState[idx], 8)) {
	//	return true;
	//}

	return false;

}

bool RobotGroupManager::robot_idle(int idx) {

	// 无运动，轨迹完成，无运动缓冲
	if (statusList[idx].lowerStatus == 0 && statusList[idx].lineNum == robotList[idx]->get_lineNum() && robotList[idx]->trajectory.trajectory_loaded()) {

		if (get_bit(coopState[idx], 7) == 0) {
			LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << idx << " task complete.");
			set_bit(coopState[idx], 7, true);
		}

		return true;
	}

	set_bit(coopState[idx], 7, false);

	return false;
}


bool RobotGroupManager::robot_group_idle(const std::vector<int>& ids) {

	std::vector<int> idSet = ids;

	// 输入为空时检查所有机器人
	if (idSet.size() == 0) {
		idSet = std::vector<int>(robotList.size(), 0);
		for (size_t i = 0; i < idSet.size(); ++i) {
			idSet[i] += i;
		}
	}

	for (size_t i = 0; i < idSet.size(); ++i) {

		// 输入异常
		if (idSet[i] > robotList.size() || idSet[i] < 0) {
			return false;
		}

		int idx = idSet[i];
		// 协同机器人运动未完成
		if (statusList[idx].lineNum != robotList[idx]->get_lineNum() || statusList[idx].lowerStatus != 0) {
			return false;
		}

	}

	return true;
}

void RobotGroupManager::set_group_sync_config(int robotIdx) {

	// 当前无轨迹
	if (robotList[robotIdx]->trajectory.trajectory_loaded()) {
		syncReadyState[robotIdx] = 0;
		syncState[robotIdx].clear();
		return;
	}

	// 当前机器人下发的运动未完成
	if (statusList[robotIdx].lowerStatus != 0 || statusList[robotIdx].lineNum != robotList[robotIdx]->get_lineNum()) {
		syncReadyState[robotIdx] = 0;
		syncState[robotIdx].clear();
		return;
	}

	// 正逆解切换未完成
	//if (!kinematics_mached(robotIdx)) {
	//	syncReadyState[robotIdx] = 0;
	//	syncState[robotIdx].clear();
	//	return;
	//}

	// 当前需要下发的轨迹的同步参数
	auto curTraj = robotList[robotIdx]->trajectory.get_curTraj();
	auto synCfg = deserialize_Sync_Config(curTraj.appendix);
	syncState[robotIdx] = synCfg;

}


void RobotGroupManager::update_sync_state(int robotIdx) {

	// 等待机器人是否已就绪
	auto curSync = syncState[robotIdx];
	syncReadyState[robotIdx] = 1;
	
	for (auto& ite = curSync.map.begin(); ite != curSync.map.end(); ++ite) {
		// 同步类型
		int type = ite->first;

		// 等待激活
		if (type == 4) {
		}
		// 等待同步 / 协同
		else if (type == 2 || type == 3) {

			for (auto& syncPair : ite->second) {
				int idx = syncPair.first;
				int num = syncPair.second;

				// 协同的中间轨迹无需等待
				if (type == 3 && num <= 0) {
					break;
				}

				// 匹配机器人状态
				auto oppositeSync = syncState[idx];

				// 未匹配到相同的同步类型
				auto findSyncType = oppositeSync.map.find(type);
				if (findSyncType == oppositeSync.map.end()) {
					syncReadyState[robotIdx] = 0;
				}
				else {
					bool syncMatch = false;
					for (auto& oppositePair : findSyncType->second) {
						if (oppositePair.first == robotIdx && oppositePair.second == num) {
							syncMatch = true;
							break;
						}
					}

					// 等待的机器人同步号不匹配
					if (!syncMatch)
						syncReadyState[robotIdx] = 0;
				}
			}

		}
	}

}


bool RobotGroupManager::robot_sync_ready(int robotIdx) {

	// 当前同步
	auto curSync = syncState[robotIdx];

	// 当前轨迹同步
	auto curTraj = robotList[robotIdx]->trajectory.get_curTraj();
	auto synCfg = deserialize_Sync_Config(curTraj.appendix);

	// 无同步号
	if (synCfg.map.empty()) {
		return true;
	}

	// 新的轨迹同步有变化
	auto curSyncSerial = serialize_Sync_Config(curSync);
	auto synCfgSerial = serialize_Sync_Config(synCfg);
	if (curSync.different_from(synCfg))
		return false;


	// 协同机器人已就绪
	for (auto& ite = curSync.map.begin(); ite != curSync.map.end(); ++ite) {
		// 同步类型
		int type = ite->first;

		// 等待激活
		if (type == 4) {
			for (auto& syncPair : ite->second) {
				if (waitState[robotIdx].find(syncPair.second) == waitState[robotIdx].end())
					return false;
			}
		}
		// 等待同步 / 协同
		else if (type == 2 || type == 3) {

			for (auto& syncPair : ite->second) {
				int idx = syncPair.first;
				int num = syncPair.second;

				// 协同的中间轨迹无需等待
				if (type == 3 && num <= 0) {
					return true;
				}

				// 匹配机器人状态
				auto oppositeSync = syncState[idx];

				// 未匹配到相同的同步类型
				auto findSyncType = oppositeSync.map.find(type);
				if (findSyncType == oppositeSync.map.end()) {
					return false;
				}
				else {
					bool syncMatch = false;
					for (auto& oppositePair : findSyncType->second) {
						if (oppositePair.first == robotIdx && oppositePair.second == num) {
							syncMatch = true;
							break;
						}
					}

					// 机器人同步号不匹配
					if (!syncMatch)
						return false;

					// 等待同步的机器人还在等待其他机器人
					if (syncReadyState[idx] == 0)
						return false;
				}
			}

		}
	}

	// 协同就绪
	auto synCfgData = serialize_Sync_Config(synCfg).second;
	if (synCfgData.size() > 1) {
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << robotIdx << " sync ready: " <<
			vector_to_string(synCfgData, 2));
	}
	return true;
}


void RobotGroupManager::correct_sync_speed() {

	std::vector<int> visit(robotList.size(), 0);

	for (size_t i = 0; i < robotList.size(); ++i) {
		// 已经被修正
		if (visit[i] == 1)
			continue;

		// 当前无轨迹
		if (robotList[i]->trajectory.trajectory_loaded())
			continue;

		// 当前轨迹
		auto curTraj = robotList[i]->trajectory.get_curTraj();
		auto synCfg = deserialize_Sync_Config(curTraj.appendix);
		auto findSync = synCfg.map.find(3);

		visit[i] = 1;
		if (findSync == synCfg.map.end())
			continue;
		visit[findSync->second[0].first] = 1;

		// 协同就绪
		if (robot_sync_ready(i) && findSync->first > 0) {

			// 协同段轨迹总时间
			float curTime = robotList[i]->trajectory.get_curTraj().get_syncDist();

			// 从机序号
			int idx = findSync->second[0].first;
			float time = robotList[idx]->trajectory.get_curTraj().get_syncDist();

			if (curTime < time) {
				idx = i;
			}
			else {
				time = curTime;
			}
			// 速度修正日志
			LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << idx << " correct duration "
				<< robotList[idx]->trajectory.get_curTraj().get_syncDist()
				<< " to " << time);

			// 遍历轨迹，修正速度
			for (auto& traj : robotList[idx]->trajectory.trajList) {
				// 仅修正当前协同段
				auto tmpCfg = deserialize_Sync_Config(traj.appendix);
				auto tmpFind = tmpCfg.map.find(3);
				if (tmpFind == tmpCfg.map.end()) {
					break;
				}

				// 关节运动不修正
				if (traj.isCartesian() && time > 0) {
					traj.speed *= robotList[idx]->trajectory.get_curTraj().get_syncDist() / time;
				}

				// 结束协同，结束修正
				if (tmpFind->second[0].second < 0)
					break;
			}

		}

	}

	return;
}


void RobotGroupManager::robot_in_place_command(int robotIdx) {

	// 无已下发轨迹
	if (trajHistory[robotIdx].empty())
		return;

	// 当前已完成轨迹编号
	int lineNum = statusList[robotIdx].lineNum;
	// 第一条历史轨迹
	auto curTraj = trajHistory[robotIdx].front();

	// 开始执行
	if (lineNum == curTraj.lineNum - 1 && !get_bit(coopState[robotIdx], 5)) {

		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << robotIdx << " run traj " << curTraj.lineNum << ", save seq: " << curTraj.saveSeq);

		// 触发机器人等待
		auto syncMap = deserialize_Sync_Config(curTraj.get_appendix()).map;
		if (syncMap.find(5) != syncMap.end()) {
			for (const auto& notifyItem : syncMap[5]) {
				//set_bit(waitState[notifyItem.first], notifyItem.second, 1);
				waitState[notifyItem.first].insert(notifyItem.second);
			}
		}

		// 清空到位前运动
		//Move_Action moveCfg = deserialize_Move_Action(trajHistory[robotIdx].front().get_appendix());
		//moveCfg.actionAfter.clear();
		//trajHistory[robotIdx].front().add_appendix(serialize_Move_Action(moveCfg));

		// 设置关联
		std::unordered_set<int> bindIdxSet;
		// 协同
		if (syncMap.find(3) != syncMap.end()) {
			bindIdxSet.insert(syncMap[3].front().first);
		}
		// 绑定
		if (syncMap.find(1) != syncMap.end()) {
			for (auto& pair : syncMap[1]) {
				bindIdxSet.insert(pair.first);
			}
		}
		disableGroup[robotIdx] = std::vector<int>(bindIdxSet.begin(), bindIdxSet.end());
		if (disableGroup[robotIdx].size() > 0) {
			LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << robotIdx << " disable group: "
				<< vector_to_string(disableGroup[robotIdx], 0)
			);
		}

		// 开始执行轨迹
		set_bit(coopState[robotIdx], 5, true);
	}
	// 运动完成: 机器人到位
	else if (lineNum == curTraj.lineNum) {
		// 打印到位日志
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << robotIdx << " reach point(" << curTraj.lineNum << ", " <<
			static_cast<int>(curTraj.get_trajType()) << "): " <<
			vector_to_string(curTraj.isJoint() ? statusList[robotIdx].jPos : statusList[robotIdx].cPos));

		// 轨迹完成
		set_bit(coopState[robotIdx], 5, false);
		// 历史轨迹弹出
		trajHistory[robotIdx].pop_front();
	}
	// 轨迹编号异常
	else if (lineNum > curTraj.lineNum) {
		
		while (lineNum > curTraj.lineNum) {
			// 历史轨迹弹出
			if (trajHistory[robotIdx].size() > 0) {
				trajHistory[robotIdx].pop_front();
				curTraj = trajHistory[robotIdx].front();
			}
			else {
				return;
			}
		}
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << robotIdx << " reach point(" << curTraj.lineNum << ", " <<
			static_cast<int>(curTraj.get_trajType()) << "): " <<
			vector_to_string(curTraj.isJoint() ? statusList[robotIdx].jPos : statusList[robotIdx].cPos));
	}

}


int RobotGroupManager::pause_coop_robot(int idx) {

	// 暂停关联机器人
	for (auto& robot : disableGroup[idx]) {
		// 关联机器人未暂停
		if ((statusList[robot].upperStatus >> 2) > 0) {
			robotList[robot]->task_pause();
			LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << robot << " paused because of error in robot " << idx);
			// 标记为被动暂停
			robotList[robot]->set_upperStatus(0x02);
		}
	}

	return 0;
}


int RobotGroupManager::calc_sync_duration(int robotIdx) {

	auto preTraj = robotList[robotIdx]->trajectory.get_preTraj();
	// 协同开始、结束轨迹迭代器
	auto begTraj = robotList[robotIdx]->trajectory.trajList.begin();
	auto endTraj = robotList[robotIdx]->trajectory.trajList.begin();
	float segmDist = 0.0;

	for (auto curTraj = robotList[robotIdx]->trajectory.trajList.begin(); curTraj != robotList[robotIdx]->trajectory.trajList.end(); ++curTraj) {

		// 关节运动不执行
		if (curTraj->isJoint())
			break;

		// 当前轨迹同步参数
		auto synCfg = deserialize_Sync_Config(curTraj->appendix);

		auto findSync = synCfg.map.find(3);
		// 当前轨迹有协同，且协同轨迹时间未计算
		if (findSync != synCfg.map.end() && curTraj->get_syncDist() < 1e-2) {

			// 当前轨迹长度
			float curDist = 0.0;
			if (curTraj->isCartesian()) {
				auto trajInfo = calc_traj_info(preTraj.mainPoint, curTraj->auxPoint, curTraj->mainPoint, curTraj->isArc());
				curDist = trajInfo[3];
				curDist /= curTraj->get_speed();
			}

			// 协同开始
			if (findSync->second.front().second > 0) {
				segmDist = curDist;
				begTraj = curTraj;
			}
			else {
				// 沿用协同
				segmDist += curDist;
			}
			endTraj = curTraj;

			// 取消协同 / 下一条无轨迹 / 下一条为关节
			endTraj++;
			if (findSync->second.front().second < 0
				|| (endTraj == robotList[robotIdx]->trajectory.trajList.end())
				|| endTraj->isJoint()) {

				//if (findSync->second.front().second < 0)
				//	endTraj++;

				for (auto& ite = begTraj; ite != endTraj; ++ite) {
					ite->set_syncDist(segmDist);
				}
			}
		}

		preTraj.mainPoint = curTraj->mainPoint;
	}

	return 0;
}



int RobotGroupManager::robot_group_resume(int idx) {
	// 当前机器人继续
	robotList[idx]->task_resume();
	// 暂停触发标志复位
	set_bit(coopState[idx], 8, false);

	// 同步继续绑定机器人
	for (auto& robot : disableGroup[idx]) {
		robotList[robot]->task_resume();
	}
	return 0;
}

int RobotGroupManager::robot_group_resume(const std::vector<int>& idxList) {
	for (auto& idx : idxList) {
		// 当前机器人继续
		robotList[idx]->task_resume();

		// 同步继续绑定机器人
		for (auto& robot : disableGroup[idx]) {
			robotList[robot]->task_resume();
		}
	}
	return 0;
}

int RobotGroupManager::robot_group_pause(int idx) {
	// 当前机器人暂停
	robotList[idx]->task_pause();
	// 暂停触发标志置位
	set_bit(coopState[idx], 8, true);

	// 同步暂停绑定机器人
	for (auto& robot : disableGroup[idx]) {
		robotList[robot]->task_pause();
	}

	return 0;
}

int RobotGroupManager::robot_group_pause(const std::vector<int>& idxList) {
	for (auto& idx : idxList) {
		// 当前机器人暂停
		robotList[idx]->task_pause();
		
		// 同步暂停绑定机器人
		for (auto& robot : disableGroup[idx]) {
			robotList[robot]->task_pause();
		}
	}

	// 绑定机器人均暂停后更新位置
	robot_group_update_saved_pos(idxList);

	return 0;
}


int RobotGroupManager::robot_group_update_saved_pos(const std::vector<int>& idxList) {
	bool paused = false;

	while (!paused) {
		paused = true;
		for (auto& idx : idxList) {
			// 运动中且未暂停
			if (get_bit(statusList[idx].lowerStatus, 0) == 1 && get_bit(statusList[idx].lowerStatus, 1) == 0) {
				paused = false;
				break;
			}
		}
		if (!paused) {
			// 延时
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}

	// 暂停成功，保存状态
	for (auto& idx : idxList) {
		robotList[idx]->trigger_action(6, {});
	}
	return 0;
}

int RobotGroupManager::robot_group_clear_task(int idx) {
	// 暂停触发标志复位
	set_bit(coopState[idx], 8, false);

	robotList[idx]->task_stop();

	return 0;
}

int RobotGroupManager::robot_group_stop(int idx) {
	// 暂停触发标志复位
	set_bit(coopState[idx], 8, false);

	robotList[idx]->emergency_stop();

	return 0;
}


void RobotGroupManager::reset_wait_state(int robotIdx) {

	// 当前轨迹同步
	auto curTraj = robotList[robotIdx]->trajectory.get_curTraj();
	auto synCfg = deserialize_Sync_Config(curTraj.appendix);

	// IO等待标志复位
	if (synCfg.map.find(4) != synCfg.map.end()) {

		for (auto& syncPair : synCfg.map[4]) {
			waitState[robotIdx].erase(syncPair.second);
		}

	}

}


std::string vector_to_string(const std::vector<float>& data, int fixed) {
	std::string str;
	if (data.size() < 1) {
		return str;
	}

	for (int i = 0; i < data.size(); ++i) {
		auto dataStr = std::to_string(data[i]);

		if (fixed > 0) {
			str += dataStr.substr(0, dataStr.find(".") + fixed + 1);
		}
		else if (fixed == 0) {
			str += dataStr.substr(0, dataStr.find("."));
		}
		else {
			str += dataStr;
		}

		if (i < data.size() - 1) {
			str += ", ";
		}
	}

	return str;
}

std::string vector_to_string(const std::vector<int>& data, int fixed) {
	std::vector<float> ans(data.size(), 0);

	for (size_t i = 0; i < data.size(); ++i) {
		ans[i] = static_cast<int>(data[i]);
	}

	return vector_to_string(ans, 0);
}

int set_bit(int& state, int idx, bool enable) {

	// 判断总共有多少位

	// 位掩码
	int mask = 0xFFFFFF - (1 << idx);
	state = (state & mask) + (enable << idx);

	return 0;
}

int get_bit(int state, int idx) {
	return (state >> idx) % 2;
}

} // namespace ZMotionRobot
