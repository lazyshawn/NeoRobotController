#pragma once

#include <algorithm>
#include <chrono>
#include <deque>
#include <mutex>
#include <vector>

// 缓存区数据格式
struct BufferUnit {
	long long timeStamp;

	// 运动标志
	int runState;

	// 位置
	std::vector<float> dpos;

public:
	BufferUnit(long long stamp, int flag, const std::vector<float>& data);
};

/* **********************************************************
* @brief  上下位机缓冲数据同步
*
* Step:
* 1. 对齐上下位机时间戳
* 2. 缓存数据包
* 3. 查询/插补计算给定上位机时间下的实时数据
*********************************************************** */
class BufferSynchronizer {
	//! 最大缓存数据长度
	int maxBuffLen = 100;
	// ! 最大插补间隙
	long long maxDetT;
	//! 下位机时间参考基准(任意毫秒为单位的longlong类型)
	long long slaveTimeBase;
	//! 上位机时间参考基准
	std::chrono::time_point<std::chrono::steady_clock> materTimeBase;

	// 互斥锁与条件变量 
	std::mutex mtxBuffer;
	std::condition_variable cvBuffer;

	//! 下位机缓冲区数据
	std::deque<BufferUnit> slaveBuffer;
	//! 上位机缓冲区数据
	std::deque<BufferUnit> masterBuffer;

public:
	int set_maxBuffLen(int len);

	// 时间戳同步
	int stamp_synchronize(std::chrono::time_point<std::chrono::steady_clock> masterStamp, long long slaveStamp);

	// 上位机时间戳转化为下位机时间戳
	long long to_slave_time(long long masterStamp) const;

	// 下位机时间戳转化为上位机时间戳
	long long to_master_time(long long slaveStamp) const;

	// 新增下位机数据
	int push_slave_buffer(long long slaveStamp, int flag, const std::vector<float>& data);

	// 新增上位机数据
	int push_master_buffer(long long masterStamp, const std::vector<float>& data);

	// 按下位机时间查询下位机数据
	int query_slave_buffer(long long slaveStamp, std::vector<float>& data) const;
	// 按下位机时间查询下位机数据
	int query_slave_buffer(int idx, float value, std::vector<float>& data, float maxDist) const;

	// 弹出最新数据
	int pop_new_buffer(int num, bool popFlag, std::vector<BufferUnit>& buffer);

};
