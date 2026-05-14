
#include "common/TrajectorySegment.h"

/***********************************************************************
 *                        M O T I O N C F G T Y P E                    *
 ***********************************************************************/
MotionCfgType SwingConfig::type = MotionCfgType::SWING;

void SwingConfig::clear() {
	enable = 0;
	freq = 0;
	leftWidth = 0;
	rightWidth = 0;

	return;
}

int SwingConfig::get_type() const {
	return static_cast<int>(type);
}

int SwingConfig::serialize(std::vector<double>& param) const {
	param.clear();
	// 设置参数
	param.push_back(enable);
	param.push_back(freq);
	param.push_back(leftWidth);
	param.push_back(rightWidth);

	return static_cast<int>(type);
}

int SwingConfig::deserialize(const std::vector<double>& param) {
	// 参数不全，使用默认参数
	if (param.size() < 4) {
		this->clear();
		return 1;
	}

	enable = static_cast<int>(param[0]);
	freq = param[1];
	leftWidth = param[2];
	rightWidth = param[3];

	return 0;
}

std::vector<double> PosData::all_to_vector() {
	std::vector<double> ans = rbtPos;
	ans.insert(ans.end(), extPos.begin(), extPos.end());
	ans.insert(ans.end(), pstPos.begin(), pstPos.end());

	return ans;
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