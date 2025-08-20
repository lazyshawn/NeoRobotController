/***************************************************************************
 * @file   RobotLogger.h
 * @biref  日志管理
 *
 * 针对使用的不同的日志库，修改日志接口。
 * Example:
 *   class RobotLog {
 *   }
 *   RobotLog logger;
 *   RBT_LOG_INFO(RobotLog::getLogger(), logEvent);
 *
 ****************************************************************************/

#pragma once

#include <vector>
#include <string>

#include "log4cplus/log4cplus.h"
#include <log4cplus/logger.h>

#define RBT_LOG_INFO(logger, logEvent) LOG4CPLUS_INFO(logger, logEvent)

namespace FSAIRobotInterface {

/**
* @brief  机器人日志类
*/
class RobotLog {
public:
	inline static log4cplus::Logger& getLogger() {
		static RobotLog clog;
		return  clog.logger;
	}

private:
	static log4cplus::Logger logger;

	RobotLog();
	RobotLog(const RobotLog& log) = delete;
	RobotLog& operator=(const RobotLog& log) = delete;
};


/**
* @brief  控制卡日志类
*/
class ControllerLog {
public:
	inline static log4cplus::Logger& getLogger() {
		static ControllerLog ctrLog;
		return  ctrLog.logger;
	}

private:
	static log4cplus::Logger logger;

	ControllerLog();
	ControllerLog(const ControllerLog& log) = delete;
	ControllerLog& operator=(const ControllerLog& log) = delete;
};


/**
* @brief  序列化数据转字符串
* @param  data   序列化数据
* @param  fixed  保留小数点位数, -1:默认
* @return 以逗号分隔的字符串
*/
std::string vector_to_string(const std::vector<float>& data, int fixed = -1);
std::string vector_to_string(const std::vector<int>& data, int fixed = -1);


} // namespace ZMotionRobot
