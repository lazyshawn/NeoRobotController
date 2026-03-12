#pragma once
/* ************************************************************ *
* @brief 插补轨迹缓存的实现                                     *
*															    *
* 主要功能如下：											    *
* 1. 提供管理插补轨迹缓存的对象                      	        *
* ************************************************************* */

#include "interpolation/InterpSegment.h"

// 插补器缓存数据
class InterpBuffer {
	//! 点位指令缓存数组
	std::vector<std::shared_ptr<InterpSegment>> interpBuf;
	//! 缓冲容量(N)
	int maxBufNum;
	//! 预留余量(n)
	int reserveNum;
	//! 缓存起点编号, 当前正在插补的轨迹编号, [0,...]
	int bufBeg;
	//! 缓存终点编号, 下一条写入缓冲的轨迹编号, [beg, beg+N-n]
	int bufEnd;
	//! 队尾缓冲正在使用标识符
	bool bufOccupied = false;

	//! 任务参数，独立于单条轨迹的全局轨迹参数，主要包含支持实时修改运动参数
	TaskParam taskParam;

	// 插补周期(s)
	double cycleTime = 4e-3;

	// 获取相邻的轨迹索引 [0,N)
	int get_neighbor_index(int *pre, int *cur, int *next);
	// 获取相邻的轨迹指针, 注意C++中形参的指针是值传递，需要改为指针引用
	int get_neighbor_buffer(int cur, InterpSegment *&preBuf, InterpSegment *&curBuf, InterpSegment *&nextBuf);
	// 获取后续轨迹指针
	InterpSegment *get_following_buffer(int cur, int accent);

	// --- 关节轨迹处理
	int joint_prehandle();
	int joint_plane();
	int joint_move();

	// --- 空间轨迹处理
	int cartesian_prehandle();
	int cartesian_plan();
	int cartesian_move();

	// 轨迹前瞻: 输出最大终点速度
	double cartesian_look_ahead();

public:
	InterpBuffer();
	~InterpBuffer();

	double get_cycleTime();

	// 可以插入轨迹点: 队尾缓冲空闲，缓冲未满, 缓冲队尾行号为负(缓冲未满自动满足)
	bool buffer_ready();

	/**
	* @brief  插入点位，并进行预处理
	* @param  pointInfo    点位信息
	* @param  motionCfg    基础运动参数
	* @param  moveCmd      缓冲指令
	* @return 异常码
	*   - 0   正常返回
	*/
	int add_move_point(const PointInfo& point, const MotionCfg& cfg, const MoveCmd& cmd);

	// 修改轨迹起点
	int set_begin_pos(const PosData& begPos);

	// 弹出队首轨迹
	int pop_front();

	// 获取缓冲队首编号
	int get_bufbeg() const;
	// 获取缓存个数
	int get_buffer_size() const;
	// 获取当前目标位置
	int get_cur_dpos(PosData& pos) const;

	// 获取指定行号的轨迹类型
	InterpSegmentType get_segment_type(int bufNum);


	/**
	* @brief  执行轨迹插补, 每段轨迹第一点自动规划并插补
	* @param  pos    插补结果
	* @return 插补完成标识
	*   - 0   插补过程中
	*   - 1   插补完成
	*   - <0  插补异常
	*/
	int move(PosData& pos);
};
