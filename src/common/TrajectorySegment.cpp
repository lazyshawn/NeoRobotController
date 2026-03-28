
#include "TrajectorySegment.h"

/***********************************************************************
 *                        M O T I O N C F G T Y P E                    *
 ***********************************************************************/
MotionCfgType SwingInterpParam::type = MotionCfgType::SWING;

void SwingInterpParam::clear() {
	enable = 0;
	freq = 0;
	leftWidth = 0;
	rightWidth = 0;

	state = 0;
	time = 0;
	duration = 0;

	return;
}

int SwingInterpParam::get_type() const {
	return static_cast<int>(type);
}

int SwingInterpParam::serialize(std::vector<double>& param) const {
	param.clear();
	// 设置参数
	param.push_back(enable);
	param.push_back(freq);
	param.push_back(leftWidth);
	param.push_back(rightWidth);

	// 过程参数
	param.push_back(state);
	param.push_back(time);
	param.push_back(duration);
	param.push_back(pos);

	return static_cast<int>(type);
}

int SwingInterpParam::deserialize(const std::vector<double>& param) {
	// 参数不全，使用默认参数
	if (param.size() < 8) {
		this->clear();
		return 1;
	}

	enable = static_cast<int>(param[0]);
	freq = param[1];
	leftWidth = param[2];
	rightWidth = param[3];

	state = static_cast<int>(param[4]);
	time = param[5];
	duration = param[6];
	pos = param[7];

	return 0;
}


/***********************************************************************
 *                        S E G M E N T B A S E                        *
 ***********************************************************************/
int SegmentBase::set_data(const PointInfo& point, const MotionCfg& cfg, const MoveCmd& cmd) {
	pointInfo = point;
	motionCfg = cfg;
	moveCmd = cmd;

	return 0;
}