#include "robot_interface/BufferSynchronizer.h"


BufferUnit::BufferUnit(long long stamp, int flag, const std::vector<float>& data) {
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

int BufferSynchronizer::push_slave_buffer(long long slaveStamp, int flag, const std::vector<float>& data) {
	BufferUnit unit(slaveStamp, flag, data);

	std::lock_guard<std::mutex> lock(mtxBuffer);

	while (slaveBuffer.size() >= maxBuffLen) {
		slaveBuffer.pop_front();
	}

	slaveBuffer.push_back(unit);

	return 0;
}

int BufferSynchronizer::query_slave_buffer(long long slaveStamp, std::vector<float>& data) const {
	return 0;
}


int BufferSynchronizer::pop_new_buffer(int num, bool popFlag, std::vector<BufferUnit>& buffer) {
	if (num <= 0)
		return 0;

	buffer.clear();
	buffer.reserve(num);

	std::lock_guard<std::mutex> lock(mtxBuffer);

	// 计算实际要读取的元素数量
	int count = std::min(num, static_cast<int>(slaveBuffer.size()));

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
