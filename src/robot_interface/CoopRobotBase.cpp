#include "robot_interface/CoopRobotBase.h"

#include "RobotLogger.h"

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
		<< "Version:         1.0.2\n"
		<< "Release Date:    260325_0945");
}

/* *************************** RobotBase *************************** */
RobotBase::RobotBase() {
}
RobotBase::~RobotBase() {
	return;
}

int RobotBase::wait_auto_task_stop() {
	std::unique_lock<std::mutex> lock(mtxMotion);

	cvMotion.wait(lock, [&]() { return motionDone; });

	// 等待条件置反
	motionDone = false;

	int ret = -1;
	//if (get_notifyType() > 0) {
	//	ret = get_notifyType();
	//	LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " notify type effect: " << ret);
	//	set_notifyType(0);
	//	return ret;
	//}

	if (robotStatus.lowerStatus == 0 && trajectory.trajectory_loaded() && task_assigned_completed()) {
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " wake up and task list empty.");
	}
	else {
		LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " wake up, status: " <<
			robotStatus.lowerStatus << ", " << robotStatus.upperStatus << ", " << robotStatus.lineNum);
	}

	return ret;
}

int RobotBase::notify_waiting_robot() {

	// 防止虚假唤醒
	std::lock_guard<std::mutex> lock(mtxMotion);
	motionDone = true;

	// 轨迹完成唤醒
	if (notifyType == 0) {
		// 清空轨迹
		trajectory.clear();
	}

	// 唤醒线程
	cvMotion.notify_one();

	return 0;
}

int RobotBase::task_assigned_completed() {
	// 下位机行号与已下发行号一致 || trajHistory 为空
	return 0;
}

int RobotBase::get_rt_robot_status(RobotStatus& status) {

	{
		// 加锁
		std::lock_guard<std::mutex> lock(mtxMotion);

		status = robotStatus;
	}


	return 0;
}

int RobotBase::set_upperStatus(int idx, int code) {
	// 加锁
	std::lock_guard<std::mutex> lock(mtxMotion);

	robotStatus.upperStatus |= code;

	return 0;
}

int RobotBase::reset_upperStatus(int idx) {

	// 上位机状态位全部复位
	if (idx < 0) {
		{
			std::lock_guard<std::mutex> lock(mtxMotion);
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
	//ZController->get_register(get_config_idx_base(), 300, readValue, 1);

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

	// 从属设备
	cfgIdx = 250;
	cfgNum = 10;
	robotConfig.slaveDeviceID.resize(cfgNum);
	for (size_t i = 0; i < cfgNum; ++i) {
		robotConfig.slaveDeviceID[i] = readValue[cfgIdx + i];
	}

	cfgIdx = 270;
	cfgNum = 10;
	robotConfig.slaveDeviceType.resize(cfgNum);
	for (size_t i = 0; i < cfgNum; ++i) {
		robotConfig.slaveDeviceType[i] = readValue[cfgIdx + i];
	}


	return ret;
}

int RobotBase::write_register_config(const RobotConfig& config) {

	int ret = 0, cfgNum = 0, cfgIdx = 0;
	LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " write register config.");

	return ret;
}

int RobotBase::capture_controller_log() {
	std::string msg;
	//ZController->read_message(msg);
	return 0;
}

int RobotBase::reboot(const char *basPath, int mode) {
	if (mode < 0 || mode > 1)
		return -1;

	//return ZController->load_basic_project(basPath, mode);
	return 0;
}

int RobotBase::load_config(const std::string& fname) {
	//return ZController->load_config(fname);
	return 0;
}

int RobotBase::export_config(const std::string& fname) {
	//return ZController->export_config(fname);
	return 0;
}

int RobotBase::trigger_action(int type, const std::vector<double>& param) {
	return 0;
}


int RobotBase::execute_move_action(const std::vector<std::pair<int, std::vector<double>>>& actionList, int flag) {

	LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " Move_Action ");

	return 0;
}

int RobotBase::process_after_send_traj() {
	return 0;
}


int RobotBase::switch_robot_mode(int type) {
	return 0;
}


int RobotBase::rewrie_actioin_param(MoveActionConfig& actionCfg) {
	auto traj = trajectory.get_curTraj();

	return 0;
}

int RobotBase::insert_task_traj() {
	// 获取当前轨迹
	auto curTraj = trajectory.get_curTraj();
	// 获取上一条轨迹
	auto preTraj = trajectory.get_preTraj();

	return 0;
}

int RobotBase::send_command(const std::string& cmd, std::string& ack, int type) {
	char cmdbuffAck[2048];
	int ret = 0;

	return ret;
}

int RobotBase::get_slave_buffer() {

	return 0;
}


int RobotBase::synchronize_slave_buffer(uint64_t& masterStamp, uint64_t& slaveStamp) {

	return slaveStamp;
}


int RobotBase::modify_point_in_buffer(int id, const std::vector<double>& pos) {
	LOG4CPLUS_INFO(RobotLog::getLogger(), "R" << aliasId << " modify point not found rewriteId: " << id << ". " << vector_to_string(pos));
	// 未找到匹配轨迹
	return 1;
}

// 起弧成功检测
bool RobotBase::check_arc_on() {
	return 0;
}
}