#pragma once

#include <climits>
#include <vector>
#include <deque>
#include <chrono>
#include <thread>

#include "ZMotionController.h"
#include "Line_Scanner.h"

class ScannerTracker {
	//! 最大缓存长度
	int maxBuffLen = 1e3;
	//! 下位机接收指令就绪
	int slaveCmdReady;
	//! 控制器
	std::shared_ptr<Controller> ZController;
	//! 参考主运动距离
	float distRef;
	//! 上位机参考时间
	long long masterTimeRef;
	//! 下位机参考时间
	long long slaveTimeRef;

	//! TCP 数据时间缓存队列
	std::deque<long long> tcpTimeBuff;
	//! 焊缝数据时间缓存队列
	std::deque<long long> seamTimeBuff;
	//! TCP 位姿缓存队列 <x,y,z,Rx,Ry,Rz>
	std::deque<std::vector<float>> tcpBuff;
	//! 焊缝位置缓存队列 <x,y,z>
	std::deque<std::vector<float>> seamBuff;
	//! 下发指令缓存队列
	std::deque<std::vector<float>> cmdBuff;

public:
	ScannerTracker();
	ScannerTracker(ZMC_HANDLE handle_);
	ScannerTracker(std::shared_ptr<Controller> ZController_);

	~ScannerTracker();

	/**
	* @brief 清空数据
	*/
	void clear();

	/**
	* @brief 下位机时间戳同步
	* @param         time    时间戳，严格递增
	* @param         pos     TCP位置
	*/
	int synchronize();

	/**
	* @brief 读取缓存的TCP点位
	* @param[out]    posList    缓存位姿队列
	* @return        状态码
	*                     0     读取成功，正常返回
	*                     1     无可读缓存区
	*                     2     时间戳异常，数据弃用
	*                    -1     下位机数据读取异常，检查下位机连接状态
	*/
	int read_tcp_buffer(std::vector<motion::Time_Pos>& tcpBuffer);

	/**
	* @brief 下发跟踪指令
	* @param         dist     偏移量对应的主运动距离
	* @param         err      控制误差，跟踪的偏移量
	* @return        状态码
	*/
	int send_tracking_cmd(float dist, const std::vector<float>& err);

private:
	/**
	* @brief 缓存新的TCP点位
	* @param         time    时间戳，严格递增
	* @param         pos     TCP位置
	*/
	int record_tcp_pos(long long time, const std::vector<float>& pos);

	/**
	* @brief 缓存新的焊缝点位
	* @param         time         时间戳，严格递增
	* @param         posInCam     相机坐标系下的焊缝位置
	*/
	int record_seam_pos(long long time, const std::vector<float>& posInCam);

	/**
	* @brief 查找给定时间戳下的机器人位姿
	* @param         time     指定时间戳
	* @param[out]    ref      指定时间戳下的机器人位姿
	* @return        状态码
	*/
	int find_latest_tcp_reference(long long time, std::vector<float>& ref);
	
	/**
	* @brief 在焊缝缓存中查找距离指定点最近的焊缝点
	* @param         pos     世界坐标系下的指定点位置
	* @param         ref     焊缝缓存中的最近参考点
	* @return        状态码
	*                     0    找到最近点，正常返回
	*                    -1    指定点未进入焊缝缓存队列
	*                     1    指定点已超出焊缝缓存队列
	*/
	int find_cloest_seam_reference(const std::vector<float>& pos, std::vector<float>& ref);

	/**
	* @brief 评估控制误差
	* @param         pos     当前TCP位置
	* @param         err     控制误差
	* @return        状态码
	*/
	int evaluate_control_error(const std::vector<float>& pos, std::vector<float>& err);

	/**
	* @brief 时间戳转化为标准时间
	* @param         timestamp     相对于纪元时刻的时间戳
	*/
	static std::tm* gettm(long long timestamp) {
		auto milli = timestamp + (long long)8 * 60 * 60 * 1000; // 此处转化为东八区北京时间
		auto mTime = std::chrono::milliseconds(milli);
		auto tp = std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>(mTime);
		auto tt = std::chrono::system_clock::to_time_t(tp);
		std::tm* now = gmtime(&tt);

		//printf("%4d年%02d月%02d日 %02d:%02d:%02d\n", now->tm_year + 1900, now->tm_mon + 1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);
		return now;
	}
};
