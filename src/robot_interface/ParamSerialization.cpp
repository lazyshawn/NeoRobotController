
#include "robot_interface/ParamSerialization.h"

namespace FSAIRobotInterface {


// ------- 摆焊参数 -------
std::pair<int, std::vector<float>> serialize_Weave(const Weave& waveCfg) {

	std::pair<int, std::vector<float>> ans;
	std::vector<float> param(13);

	param[0] = waveCfg.Id;
	param[1] = waveCfg.Shape;
	param[2] = waveCfg.Freq;
	param[3] = waveCfg.LeftWidth;
	param[4] = waveCfg.RightWidth;
	param[5] = waveCfg.Dwell_type;
	param[6] = waveCfg.Dwell_left;
	param[7] = waveCfg.Dwell_right;
	param[8] = waveCfg.Dwell_center;
	// 三角摆
	param[9]  = waveCfg.Length;
	param[10] = waveCfg.Bias;
	param[11] = waveCfg.Angle_Ltype_top;
	param[12] = waveCfg.Angle_Ltype_btm;

	ans.first = static_cast<int>(AppendixType::WAVE_CFG);
	ans.second = param;

	return ans;

}

Weave deserialize_Weave(const std::map<int, std::vector<float>>& appendix) {
	Weave cfg;
	auto ite = appendix.find(static_cast<int>(AppendixType::WAVE_CFG));
	if (ite == appendix.end())
		return cfg;
	std::vector<float> param = ite->second;

	cfg.Id          = param[0];
	cfg.Shape       = param[1];
	cfg.Freq        = param[2];
	cfg.LeftWidth   = param[3];
	cfg.RightWidth  = param[4];
	cfg.Dwell_type  = param[5];
	cfg.Dwell_left  = param[6];
	cfg.Dwell_right = param[7];
	cfg.Dwell_center = param[8];
	// 三角摆
	cfg.Length          = param[9];
	cfg.Bias            = param[10];
	cfg.Angle_Ltype_top = param[11];
	cfg.Angle_Ltype_btm = param[12];

	return cfg;
}


// ------- 焊接参数 -------
std::pair<int, std::vector<float>> serialize_Arc_WeldingParaItem(const Arc_WeldingParaItem& weldCfg) {

	std::pair<int, std::vector<float>> ans;
	std::vector<float> param;

	// 麦格米特一元模式使用电压修正
	bool useVtgCorrection = weldCfg.Id == 1 && ((weldCfg.WeldingWorkMode >> 4) % 2 == 0);

	int mode;
	float current, voltage, inductance;
	float arcTime, blowTime;

	// 焊接参数 0
	mode = weldCfg.WeldingWorkMode;
	current = weldCfg.WeldingCrt_Spd;
	voltage = useVtgCorrection ? weldCfg.VtgUniCorrection + 30 : weldCfg.WeldingVtg_Strth;
	inductance = weldCfg.Inductance;
	param.push_back(weldCfg.Id);                   // 0 起弧标志
	param.push_back(current);                      // 1 焊接电流
	param.push_back(voltage);                      // 2 焊接电压: 下发电压/修正，往同一个地址下发的，不区分两个变量
	param.push_back(mode);                         // 3 焊接工作模式
	param.push_back(inductance);                   // 4 焊接电压修正值 -> 电感

	// 起弧参数 5
	mode = weldCfg.ArcOnWorkMode;
	current = weldCfg.ArcOnCrt_Spd;
	voltage = useVtgCorrection ? weldCfg.ArcOnVtg_Correction + 30 : weldCfg.ArcOnVtg_Strth;
	arcTime = weldCfg.ArcOnTime;
	blowTime = weldCfg.ArcOnBlowTime;
	param.push_back(mode);                         // 0 起弧模式
	param.push_back(current);                      // 1 起弧电流
	param.push_back(voltage);                      // 2 起弧电压(修正)
	param.push_back(arcTime);                      // 3 起弧时间
	param.push_back(inductance);                   // 4 起弧电感
	param.push_back(blowTime);                     // 5 引气时间

	// 收弧参数 11
	mode = weldCfg.ArcOffWorkMode;
	current = weldCfg.ArcOffCrt_Spd;
	voltage = useVtgCorrection ? weldCfg.ArcOffVtg_Correction + 30 : weldCfg.ArcOffVtg_Strth;
	arcTime = weldCfg.ArcOffTime;
	blowTime = weldCfg.ArcOffBlowTime;
	param.push_back(mode);	                      // 0 收弧模式
	param.push_back(current);		              // 1 收弧电流
	param.push_back(voltage); 	                  // 2 收弧电压(修正)
	param.push_back(arcTime); 		              // 3 收弧时间
	param.push_back(inductance);                  // 4 收弧电感
	param.push_back(blowTime);                    // 5 收气时间

	ans.first = static_cast<int>(AppendixType::WELD_CFG);
	ans.second = param;

	return ans;

}

Arc_WeldingParaItem deserialize_Arc_WeldingParaItem(const std::map<int, std::vector<float>>& appendix) {
	Arc_WeldingParaItem cfg;
	auto ite = appendix.find(static_cast<int>(AppendixType::WELD_CFG));
	if (ite == appendix.end())
		return cfg;
	std::vector<float> param = ite->second;
	int num = 0;


	// 焊接参数 0
	cfg.Id = param[num++];                     // 0 起弧标志
	cfg.WeldingCrt_Spd   = param[num++];       // 1 焊接电流
	cfg.WeldingVtg_Strth = param[num++];       // 2 焊接电压
	cfg.VtgUniCorrection = param[num] - 30;    // 2 焊接电压修正值
	cfg.WeldingWorkMode  = param[num++];       // 3 焊接工作模式
	cfg.Inductance       = param[num++];       // 4 焊接电感

	// 起弧参数 5
	cfg.ArcOnWorkMode       = param[num++];      // 0 起弧模式
	cfg.ArcOnCrt_Spd        = param[num++];	     // 1 起弧电流
	cfg.ArcOnVtg_Strth      = param[num++];	     // 2 起弧电压
	cfg.ArcOnVtg_Correction = param[num] - 30;	 // 2 起弧电压修正值
	cfg.ArcOnTime           = param[num++];	     // 3 起弧时间
	cfg.Inductance          = param[num++];	     // 4 起弧电感
	cfg.ArcOnBlowTime       = param[num++];	     // 5 引气时间

	// 收弧参数 11
	cfg.ArcOffWorkMode       = param[num++];       // 0 收弧模式
	cfg.ArcOffCrt_Spd        = param[num++];       // 1 收弧电流
	cfg.ArcOffVtg_Strth      = param[num++];       // 2 收弧电压
	cfg.ArcOffVtg_Correction = param[num] - 30;    // 2 收弧电压修正值
	cfg.ArcOffTime           = param[num++];       // 3 收弧时间
	cfg.Inductance           = param[num++];       // 4 收弧电感
	cfg.ArcOffBlowTime       = param[num++];       // 5 收气时间

	return cfg;
}


// ------- 再起弧参数 -------
std::pair<int, std::vector<float>> serialize_ReArc(const ReArc& cfg) {
	std::pair<int, std::vector<float>> ans;
	std::vector<float> param;

	param.push_back(cfg.ReArc_Enable);   // 0 再起弧使能
	param.push_back(cfg.ReArcCount);     // 1 再起弧次数
	param.push_back(cfg.ReArcTime);	     // 2 再起弧时间
	param.push_back(cfg.ReArcSnagTime);  // 3 再起弧抽丝时间

	ans.first = static_cast<int>(AppendixType::REARC_CFG);
	ans.second = param;
	return ans;
}

ReArc serialize_ReArc(const std::map<int, std::vector<float>>& appendix) {
	ReArc cfg;
	auto ite = appendix.find(static_cast<int>(AppendixType::REARC_CFG));
	if (ite == appendix.end())
		return cfg;
	std::vector<float> param = ite->second;
	int num = 0;

	cfg.ReArc_Enable = param[num++];   // 0 再起弧使能
	cfg.ReArcCount = param[num++];     // 1 再起弧次数
	cfg.ReArcTime = param[num++];	   // 2 再起弧时间
	cfg.ReArcSnagTime =param[num++];   // 3 再起弧抽丝时间

	return cfg;
}

// ------- 跟踪参数 -------
std::pair<int, std::vector<float>> serialize_Track(const Track& trackCfg) {

	std::pair<int, std::vector<float>> ans;
	std::vector<float> param(20);

	param[7] = trackCfg.Id;
	// 左右跟踪参数
	param[0] = trackCfg.Lr_enable;
	param[1] = trackCfg.Lr_offset;
	param[2] = trackCfg.Lr_gain;
	// 积分常数
	param[3] = trackCfg.Lr_maxSingleCompensation;
	param[4] = trackCfg.Lr_minCompensation;
	param[5] = trackCfg.Lr_maxCompensation;
	param[6] = trackCfg.Lr_MaxCorrectAngle;
	// 上下跟踪参数
	param[10] = trackCfg.Ud_enable;
	param[11] = trackCfg.Ud_offset;
	param[12] = trackCfg.Ud_gain;
	param[14] = trackCfg.Ud_minCompensation;
	param[15] = trackCfg.Ud_maxCompensation;
	param[16] = trackCfg.Ud_MaxCorrectAngle;
	// 其他参数
	//param[18] = trackCfg.SegCorrectCycles;
	//param[19] = trackCfg.Ud_refSampleCount;

	ans.first = static_cast<int>(AppendixType::TRACK_CFG);
	ans.second = param;

	return ans;
}

Track deserialize_Track(const std::map<int, std::vector<float>>& appendix) {
	Track cfg;
	auto ite = appendix.find(static_cast<int>(AppendixType::TRACK_CFG));
	if (ite == appendix.end())
		return cfg;
	std::vector<float> param = ite->second;

	// 跟踪参数
	cfg.Id = param[7];
	// 左右跟踪参数
	cfg.Lr_enable = param[0];
	cfg.Lr_offset = param[1];
	cfg.Lr_gain = param[2];
	// 积分常数
	cfg.Lr_maxSingleCompensation = param[3];
	cfg.Lr_minCompensation = param[4];
	cfg.Lr_maxCompensation = param[5];
	cfg.Lr_MaxCorrectAngle = param[6];
	// 上下跟踪参数
	cfg.Ud_enable = param[10];
	cfg.Ud_offset = param[11];
	cfg.Ud_gain = param[12];
	cfg.Ud_minCompensation = param[14];
	cfg.Ud_maxCompensation = param[15];
	cfg.Ud_MaxCorrectAngle = param[16];
	// 其他参数
	//cfg.SegCorrectCycles = param[18];
	//cfg.Ud_refSampleCount = param[19];

	return cfg;
}


// ------- 缓冲动作参数 -------
std::pair<int, std::vector<float>> serialize_Move_Action(const Move_Action& moveCfg) {

	std::pair<int, std::vector<float>> ans;
	std::vector<float> param;

	// 运动前动作
	param.push_back(moveCfg.actionBefore.size());
	for (auto action : moveCfg.actionBefore) {
		param.push_back(action.first);
		param.push_back(action.second.size());
		param.insert(param.end(), action.second.begin(), action.second.end());
	}

	// 运动后动作
	//param.push_back(moveCfg.actionAfter.size());
	//param.insert(param.end(), moveCfg.actionAfter.begin(), moveCfg.actionAfter.end());
	param.push_back(moveCfg.actionAfter.size());
	for (auto action : moveCfg.actionAfter) {
		param.push_back(action.first);
		param.push_back(action.second.size());
		param.insert(param.end(), action.second.begin(), action.second.end());
	}

	ans.first = static_cast<int>(AppendixType::MOTION_CFG);
	ans.second = param;

	return ans;
}

Move_Action deserialize_Move_Action(const std::map<int, std::vector<float>>& appendix) {

	Move_Action cfg;
	auto ite = appendix.find(static_cast<int>(AppendixType::MOTION_CFG));
	if (ite == appendix.end())
		return cfg;
	std::vector<float> param = ite->second;
	
	int actionSize = param[0], beg = 1, end = 1;
	for (size_t i = 0; i < actionSize; ++i) {
		std::pair<int, std::vector<float>> action;
		// 动作类型
		action.first = param[beg++];

		// 参数数量
		int num = param[beg++];
		// 参数结束位置
		end = beg + num;
		action.second = std::vector<float>(param.begin() + beg, param.begin() + end);

		beg = end;
		cfg.actionBefore.push_back(action);
	}

	actionSize = param[beg++];
	for (size_t i = 0; i < actionSize; ++i) {
		std::pair<int, std::vector<float>> action;
		// 动作类型
		action.first = param[beg++];

		// 参数数量
		int num = param[beg++];
		// 参数结束位置
		end = beg + num;
		action.second = std::vector<float>(param.begin() + beg, param.begin() + end);

		beg = end;
		cfg.actionAfter.push_back(action);
	}

	return cfg;
}


// ------- 同步参数 -------
std::pair<int, std::vector<float>> serialize_Sync_Config(Sync_Config& syncCfg) {

	// 排序
	syncCfg.sort();

	// <mapSize, <type, size, <robot, num>>>
	std::pair<int, std::vector<float>> ans;
	std::vector<float> param(1, syncCfg.Id);

	// 同步类型个数
	param.push_back(syncCfg.map.size());

	for (const auto& unit : syncCfg.map) {
		param.push_back(unit.first);
		param.push_back(unit.second.size());
		for (size_t i = 0; i < unit.second.size(); ++i) {
			param.push_back(unit.second[i].first);
			param.push_back(unit.second[i].second);
		}
	}

	ans.first = static_cast<int>(AppendixType::Sync_CFG);
	ans.second = param;

	return ans;
}

Sync_Config deserialize_Sync_Config(const std::map<int, std::vector<float>>& appendix) {
	Sync_Config cfg;
	auto ite = appendix.find(static_cast<int>(AppendixType::Sync_CFG));
	if (ite == appendix.end()) {
		cfg.Id = 0;
		return cfg;
	}
	std::vector<float> param = ite->second;

	cfg.Id = param[0];
	int cfgSize = param[1], beg = 2, end = 2;
	for (size_t i = 0; i < cfgSize; ++i) {
		int type = param[beg++];
		int vecSize = param[beg++];
		std::vector<std::pair<int, int>> vecCfg;

		for (size_t j = 0; j < vecSize; ++j) {
			end = beg + 1;
			vecCfg.push_back(std::pair<int, int>(param[beg], param[end]));
			beg = end + 1;
		}

		cfg.map[type] = vecCfg;
	}
	
	return cfg;
}


Sync_Unit::Sync_Unit(int syncType, int robotId, int num) {
	type = syncType;
	item = std::pair<int, int>({ robotId, num });
}


int Sync_Config::add_sync_item(int syncType, int robotId, int num) {

	auto ite = map.find(syncType);

	if (ite != map.end()) {
		ite->second.push_back({ robotId, num });
	}
	else {
		map[syncType] = { std::pair<int,int>(robotId, num) };
	}

	return 0;
}


int Sync_Config::add_sync_item(const Sync_Unit item) {
	auto ite = map.find(item.type);

	if (ite != map.end()) {
		ite->second.push_back(item.item);
	}
	else {
		map[item.type] = { item.item };
	}

	return 0;
}


void Sync_Config::sort() {
	for (auto& vec : map) {
		std::sort(vec.second.begin(), vec.second.end(), [](const auto& a, const auto& b) {
			if (a.first < b.first) {
				return true;
			}
			else if (a.first > b.first) {
				return false;
			}
			else {
				return a.second < b.second;
			}
		});
	}
}

bool Sync_Config::different_from(Sync_Config& next) {

	// 需要等待的同步类型
	std::vector<int> syncType = { 2,3,4 };

	for (auto& i : syncType) {
		
		// 同步类型变化
		if (map.count(i) != next.map.count(i))
			return true;


		// 同步组变化
		for (size_t j = 0; j < map[i].size(); ++j) {
			// 等待机器ID变化
			if (map[i][j].first != next.map[i][j].first)
				return true;
			// 同步号变化
			if (map[i][j].second != next.map[i][j].second) {
				if (i != 3 || next.map[i][j].second >= 0) {
					return true;
				}
			}
		}

	}


	return false;
}

// 删除规则
int Sync_Config::clear_item(const std::vector<int>& syncType) {
	for (auto& item : syncType) {
		map.erase(item);
	}
	return 0;
}

// 需要等待
int Sync_Config::need_sync() {
	if (Id <= 0)
		return false;

	std::vector<int> syncType = { 2,3,4 };
	for (auto& type : syncType) {
		if (map.count(type) > 0) {
			return type;
		}
	}
	return false;
}

// ------- 运动参数 -------

std::pair<int, std::vector<float>> serialize_Move_Config(const Move_Config& moveCfg) {

	std::pair<int, std::vector<float>> ans;
	std::vector<float> param;

	param.push_back(moveCfg.speed);
	param.push_back(moveCfg.smooth);

	ans.first = static_cast<int>(AppendixType::MOVE_CONFIG);
	ans.second = param;

	return ans;
}

Move_Config deserialize_Move_Config(const std::map<int, std::vector<float>>& appendix) {

	Move_Config cfg;
	auto ite = appendix.find(static_cast<int>(AppendixType::MOVE_CONFIG));
	if (ite == appendix.end())
		return cfg;
	std::vector<float> param = ite->second;

	cfg.speed = param[0];
	cfg.smooth = param[1];

	return cfg;
}


} // namespace ZMotionRobot