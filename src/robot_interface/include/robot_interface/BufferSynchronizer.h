#pragma once

#include <deque>
#include <vector>

// 缓存区数据格式
class BufferUnit {
	long long timeStamp;

	// 位置
	std::vector<float> dpos;

public:
	BufferUnit(long long stamp, const std::vector<float>& data);
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
	//! 上位机时间参考基准(任意毫秒为单位的longlong类型)
	long long masterTimeBase;
	//! 下位机时间参考基准
	long long slaveTimeBase;

	//! 下位机缓冲区数据
	std::deque<BufferUnit> slaveBuffer;
	//! 上位机缓冲区数据
	std::deque<BufferUnit> masterBuffer;

public:
	// 时间戳同步
	int stamp_synchronize(long long masterStamp, long long slaveStamp);

	// 上位机时间戳转化为下位机时间戳
	long long to_slave_time(long long masterStamp) const;

	// 下位机时间戳转化为上位机时间戳
	long long to_master_time(long long slaveStamp) const;

	// 新增下位机数据
	int push_slave_buffer(long long slaveStamp, const std::vector<float>& data);

	// 新增上位机数据
	int push_master_buffer(long long masterStamp, const std::vector<float>& data);

	// 按下位机时间查询下位机数据
	int query_slave_buffer(long long slaveStamp, std::vector<float>& data) const;
	// 按下位机时间查询下位机数据
	int query_slave_buffer(int idx, float value, std::vector<float>& data, float maxDist) const;

	// 弹出最新数据
	int pop_new_buffer(int num, bool popFlag);

};
