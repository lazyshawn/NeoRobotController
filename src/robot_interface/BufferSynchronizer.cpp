#include "robot_interface/BufferSynchronizer.h"


BufferUnit::BufferUnit(long long stamp, const std::vector<float>& data) {
	timeStamp = stamp;
	dpos = data;
}

int BufferSynchronizer::push_slave_buffer(long long slaveStamp, const std::vector<float>& data) {
	BufferUnit unit(slaveStamp, data);

	while (slaveBuffer.size() >= maxBuffLen) {
		slaveBuffer.pop_front();
	}

	slaveBuffer.push_back(unit);

	return 0;
}

int BufferSynchronizer::query_slave_buffer(long long slaveStamp, std::vector<float>& data) const {
	return 0;
}