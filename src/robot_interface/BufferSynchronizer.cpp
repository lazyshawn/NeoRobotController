#include "robot_interface/BufferSynchronizer.h"


motion::BufferUnit::BufferUnit(uint64_t stamp, int flag, const std::vector<float>& data) {
	timeStamp = stamp;
	runState = flag;
	dpos = data;
}



int BufferSynchronizer::set_maxBuffLen(int len) {
	maxBuffLen = len;
	return 0;
}

int BufferSynchronizer::stamp_synchronize(std::chrono::time_point<std::chrono::steady_clock> masterStamp, long long slaveStamp) {
	materTimeBase = masterStamp;
	slaveTimeBase = slaveStamp;

	std::lock_guard<std::mutex> lock(mtxBuffer);

	slaveBuffer.clear();
	masterBuffer.clear();

	return 0;
}

int BufferSynchronizer::push_slave_buffer(uint64_t slaveStamp, int flag, const std::vector<float>& data) {
	motion::BufferUnit unit(slaveStamp, flag, data);

	std::lock_guard<std::mutex> lock(mtxBuffer);

	while (slaveBuffer.size() >= maxBuffLen) {
		slaveBuffer.pop_front();
	}

	// 时间戳校验，只插入更新的数据, 防止漏读时重复读取旧数据
	if (!slaveBuffer.empty() && slaveBuffer.back().timeStamp >= unit.timeStamp) {
		return 1;
	}

	slaveBuffer.push_back(unit);

	return 0;
}

int BufferSynchronizer::query_slave_buffer(uint64_t slaveStamp, std::vector<float>& data) const {
	return 0;
}


int BufferSynchronizer::pop_new_buffer(std::vector<motion::BufferUnit>& buffer, bool popFlag, int num) {

	buffer.clear();
	std::lock_guard<std::mutex> lock(mtxBuffer);

	// 计算实际要读取的元素数量
	int count = num <= 0 ? slaveBuffer.size() : std::min(num, static_cast<int>(slaveBuffer.size()));
	buffer.reserve(count);

	// 读取前count个元素
	auto it = slaveBuffer.begin();
	for (int i = 0; i < count; ++i) {
		buffer.push_back(*it++);
	}

	// 如果flag为true，清除已读取的元素
	if (popFlag && count > 0) {
		slaveBuffer.erase(slaveBuffer.begin(), slaveBuffer.begin() + count);
	}

	return 0;
}
