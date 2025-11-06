#pragma once

#include <vector>
#include <map>
#include <algorithm>

#include "FsCraftDef.h"

namespace FSAIRobotInterface {

// 自定义参数
struct Sync_Unit {
	// 同步类型: -1:无, 1:关联, 2:同步, 3:协同, 4:等待, 5:激活
	int type = -1;
	// 同步组: <机器人ID, 同步编号>
	std::pair<int, int> item;

	Sync_Unit() {};
	Sync_Unit(int syncType, int robotId, int num);
};

struct Sync_Config {

	// <同步类型, <机器人ID，同步编号>>
	std::map<int, std::vector<std::pair<int, int>>> map;

	// 清空
	inline void clear() {
		map.clear();
	}
	// 同步组排序
	void sort();

	// 处理添加规则
	int add_sync_item(int syncType, int robotId, int num);
	int add_sync_item(const Sync_Unit item);
	// 同步参数是否相同
	bool different_from(Sync_Config& next);
};


struct Move_Action {
	//! 触发底层封装的特殊动作 <动作类型，动作参数>
	std::vector<std::pair<int, std::vector<float>>> actionBefore;
	std::vector<std::pair<int, std::vector<float>>> actionAfter;
};

struct Move_Config {
	float speed = 10;
	float smooth = -1;
};

// 自定义参数类型
enum class AppendixType {
	WAVE_CFG, TRACK_CFG, WELD_CFG, REARC_CFG, MOTION_CFG, Sync_CFG, MOVE_CONFIG
};

// 自定义参数序列化与反序列化
// 摆焊参数
std::pair<int, std::vector<float>> serialize_Weave(const Weave& waveCfg);
Weave deserialize_Weave(const std::map<int, std::vector<float>>& appendix);

// 焊接参数
std::pair<int, std::vector<float>> serialize_Arc_WeldingParaItem(const Arc_WeldingParaItem& weldCfg);
Arc_WeldingParaItem deserialize_Arc_WeldingParaItem(const std::map<int, std::vector<float>>& appendix);

// 再起弧参数
std::pair<int, std::vector<float>> serialize_ReArc(const ReArc& cfg);
ReArc serialize_ReArc(const std::map<int, std::vector<float>>& appendix);

// 跟踪参数
std::pair<int, std::vector<float>> serialize_Track(const Track& trackCfg);
Track deserialize_Track(const std::map<int, std::vector<float>>& appendix);

// 缓冲运动参数
std::pair<int, std::vector<float>> serialize_Move_Action(const Move_Action& waveCfg);
Move_Action deserialize_Move_Action(const std::map<int, std::vector<float>>& appendix);

// 协同参数
std::pair<int, std::vector<float>> serialize_Sync_Config(Sync_Config& syncCfg);
Sync_Config deserialize_Sync_Config(const std::map<int, std::vector<float>>& appendix);

// 运动参数
std::pair<int, std::vector<float>> serialize_Move_Config(const Move_Config& moveCfg);
Move_Config deserialize_Move_Config(const std::map<int, std::vector<float>>& appendix);

}
