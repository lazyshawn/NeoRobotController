
#include "SeamTracker.h"
#include <iostream>

ScannerTracker::ScannerTracker() {

}


ScannerTracker::ScannerTracker(ZMC_HANDLE handle_) {
	ZController = std::make_shared<ZMotionController>();
	ZController->set_handle(handle_);
	
	this->synchronize();
}


ScannerTracker::ScannerTracker(std::shared_ptr<ZMotionController> ZController_) {
	ZController = ZController_;
}


ScannerTracker::~ScannerTracker() {

}


void ScannerTracker::clear() {
	// 主动释放
	//ZController.reset();

	// 清空数据
	tcpTimeBuff.clear();
	seamTimeBuff.clear();
	tcpBuff.clear();
	seamBuff.clear();
}


int ScannerTracker::synchronize() {
	this->clear();

	// 设置参考时间
	slaveTimeRef = ZController->get_time_stamp();
	masterTimeRef = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

	return 0;
}


int ScannerTracker::read_tcp_buffer(std::vector<motion::Time_Pos>& tcpBuffer) {
	// 1. 读取缓存区
	std::vector<float> retValue;
	// 可读缓存区索引(1) + 下发缓存区就绪(1) + [时间戳(1) + 主运动距离(1) + tcp位姿(6)] * 10 * 2
	ZController->get_register(100000, 163, retValue);
	// 可读缓存区的索引
	int readIdx = static_cast<int>(retValue[1]);
	// 下发缓存区
	slaveCmdReady = static_cast<int>(retValue[2]);

	// 2. 数据合法性校验
	// 两个缓存区均不可读
	if (readIdx == 0) {
		return 1;
	}
	tcpBuffer = std::vector<motion::Time_Pos>(10);

	// 3. 为结构体赋值
	float preTime = 0.0;
	for (int i = 0; i < 10; ++i) {
		// 下位机时间戳
		long long slaveTime = retValue[80 * (readIdx - 1) + 3 + 8 * i];

		// 时间戳校验
		if (preTime < slaveTime) {
			return 2;
		}
		preTime = slaveTime;

		// 保存当前点矢量距离
		float dist = retValue[80 * (readIdx - 1) + 8 * i + 4];

		// 下位机时间转化为对应的上位机时间
		long long masterTime = (slaveTimeRef - slaveTime) + masterTimeRef;

		// 时间戳转换为标准格式
		auto masterTm = gettm(masterTime);
		tcpBuffer[i].time.y = masterTm->tm_year + 1900;
		tcpBuffer[i].time.m = masterTm->tm_mon + 1;
		tcpBuffer[i].time.d = masterTm->tm_mday;
		tcpBuffer[i].time.h = masterTm->tm_hour;
		tcpBuffer[i].time.mi = masterTm->tm_min;
		tcpBuffer[i].time.s = masterTm->tm_sec;
		tcpBuffer[i].time.ms = masterTime % 1000;

		// 保存实际TCP位置
		tcpBuffer[i].tcp_pos.x = retValue[80 * (readIdx - 1) + 8 * i + 5];
		tcpBuffer[i].tcp_pos.y = retValue[80 * (readIdx - 1) + 8 * i + 6];
		tcpBuffer[i].tcp_pos.z = retValue[80 * (readIdx - 1) + 8 * i + 7];
		tcpBuffer[i].tcp_pos.a = retValue[80 * (readIdx - 1) + 8 * i + 10];
		tcpBuffer[i].tcp_pos.b = retValue[80 * (readIdx - 1) + 8 * i + 9];
		tcpBuffer[i].tcp_pos.c = retValue[80 * (readIdx - 1) + 8 * i + 8];

		// 计算主运动位置：实际位置 - 偏离距离
	}

	return 0;
}


int ScannerTracker::send_tracking_cmd(float dist, const std::vector<float>& err) {
	
	// 下位机准备就绪
	if (slaveCmdReady == 0) {
		std::vector<float> cmd = { static_cast<float>(cmdBuff.size() + 1) };
		for (int i = 0; i < cmdBuff.size(); ++i) {
			cmd.insert(cmd.end(), cmdBuff.front().begin(), cmdBuff.front().end());
			cmdBuff.pop_front();
		}

		cmd.push_back(dist);
		cmd.insert(cmd.end(), err.begin(), err.end());

		// 下发缓存的补偿指令队列
		ZController->set_register(100202, cmd);
	}
	// 下位机未就绪，缓存补偿指令
	else {
		std::vector<float> cmd = { dist };
		cmd.insert(cmd.end(), err.begin(), err.end());

		cmdBuff.push_back(cmd);
	}

	return 0;
}

int ScannerTracker::record_tcp_pos(long long time, const std::vector<float>& pos) {
	if (tcpBuff.size() < maxBuffLen) {
		tcpTimeBuff.push_back(time);
		tcpBuff.push_back(pos);
	}
	else {
		tcpTimeBuff.pop_front();
		tcpTimeBuff.push_back(time);
		tcpBuff.pop_front();
		tcpBuff.push_back(pos);
	}

	return 0;
}


int ScannerTracker::record_seam_pos(long long time, const std::vector<float>& posInCam) {
	// 找到tcp缓存中距离当前点时间最近的两个点
	int preIdx = 0;
	for (int i = 0; i < tcpBuff.size(); ++i) {
		if (tcpTimeBuff[i] > time) {
			preIdx = i;
		}
		else if (tcpTimeBuff[i] <= time) {
			break;
		}
	}

	// 插值计算tcp位置
	std::vector<float> data(posInCam.size());
	float lambda = (time - tcpTimeBuff[preIdx]) / (tcpTimeBuff[preIdx+1] - tcpTimeBuff[preIdx]);
	for (int i = 0; i < posInCam.size(); ++i) {
		data[i] = (1 - lambda) * tcpBuff[preIdx][i] + lambda * tcpBuff[preIdx + 1][i];
	}

	// 计算世界坐标系下的焊缝位置

	// 缓存焊缝位置
	seamTimeBuff.push_back(time);
	seamBuff.push_back(data);

	return 0;
}


int ScannerTracker::find_cloest_seam_reference(const std::vector<float>& pos, std::vector<float>& ref) {
	int approach = 0, depart = 0;
	double preDist = std::numeric_limits<double>::max();
	for (int i = 0; i < seamBuff.size(); ++i) {
		// 缓存点到当前点的距离
		double dist = (pos[0] - seamBuff[i][0])*(pos[0] - seamBuff[i][0]) + (pos[1] - seamBuff[i][1])*(pos[1] - seamBuff[i][1]) + (pos[2] - seamBuff[i][2])*(pos[2] - seamBuff[i][2]);
		// 接近给定点
		if (approach >= 0) {
			if (preDist >= dist) {
				approach++;
			}
			else {
				approach = -approach;
			}
		}
		// 远离给定点
		else {
			if (preDist <= dist) {
				depart++;
			}
			else {
				break;
			}
			if (depart > 5) {
				break;
			}
		}
		preDist = dist;
	}

	// 连续递减或连续递增数据过短
	if (std::abs(approach) < 5 || std::abs(depart) < 5) {
		return -1;
	}

	// 估计参考点
	ref = seamBuff[-approach];

	return 0;
}


int ScannerTracker::evaluate_control_error(const std::vector<float>& pos, std::vector<float>& err) {
	std::vector<float> ref = pos;
	
	int find = find_cloest_seam_reference(pos, ref);

	if (find != 0) {
		return find;
	}

	// 计算位置偏移
	for (int i = 0; i < pos.size(); ++i) {
		err[i] = ref[i] - pos[i];
	}

	return 0;
}


