
#include "interpolation/InterpSegmentBuffer.h"
// 不同插补轨迹段的实现
#include "interpolation/SegmentJoint.h"
#include "interpolation/SegmentCartesian.h"


static const double dim_EPS = 1e-6;

/***********************************************************************
 *                        InterpBuffer                                 *
 ***********************************************************************/
InterpBuffer::InterpBuffer() {
	bufBeg = bufEnd = 0;
	maxBufNum = 10;

	for (size_t i = 0; i < maxBufNum; ++i) {
		interpBuf.push_back(std::shared_ptr<InterpSegment>(new JointInterpSegment));
	}
}
InterpBuffer::~InterpBuffer() {

}


bool InterpBuffer::buffer_ready() {
	return (!bufOccupied) && (bufEnd - bufBeg - maxBufNum < 0);
}

int InterpBuffer::add_move_point(const PointInfo& point, const MotionCfg& cfg, const MoveCmd& cmd) {
	// - 轨迹队列已满
	if (bufEnd - bufBeg - maxBufNum == 0) {
		printf("point buffer is full: %d\n", maxBufNum);
		return 1;
	}

	// 轨迹写入位置
	int curBuf = bufEnd, nextBuf, preBuf;
	get_neighbor_index(&preBuf, &bufEnd, &nextBuf);

	// 写入位置行号为非负，表示点位未执行完毕，不插入点位
	if (interpBuf[curBuf]->procInfo.lineNum >= 0) {
		return -1;
	}

	// - 插入轨迹
	bufOccupied = true;
	// 缓冲轨迹
	std::shared_ptr<InterpSegment> traj;
	// 关节轨迹
	if (cfg.moveType % 2 == 0) {
		traj = std::shared_ptr<InterpSegment>(new JointInterpSegment);
	}
	// 空间轨迹
	else {
		traj = std::shared_ptr<InterpSegment>(new CartesianInterpSegment);
	}

	// 轨迹加入缓冲
	traj->set_data(point, cfg, cmd);
	interpBuf[curBuf] = traj;

	// - 预处理
	interpBuf[curBuf]->prehandle(*interpBuf[preBuf]);

	// 行号
	interpBuf[curBuf]->procInfo.lineNum = bufEnd;

	// 若无等待则上一段终点设为当前起点
	if (bufEnd - bufBeg > 0) {
		int preBuf = (bufEnd - 1) % maxBufNum;

		interpBuf[curBuf]->pointInfo.begPos = interpBuf[preBuf]->pointInfo.endPos;
	}

	// - 轨迹插入成功
	printf("add new point: %d\n", bufEnd);
	bufEnd++;
	bufOccupied = false;

	return 0;
}

int InterpBuffer::set_begin_pos(const PosData& begPos) {
	interpBuf[bufBeg % maxBufNum]->pointInfo.begPos = begPos;
	return 0;
}

int InterpBuffer::get_bufbeg() const {
	return bufBeg;
}

// 获取缓存个数
int InterpBuffer::get_buffer_size() const {
	return bufEnd - bufBeg;
}

// 获取当前目标位置
int InterpBuffer::get_cur_dpos(PosData& pos) const {
	int idx = bufBeg % maxBufNum;

	pos = interpBuf[idx]->interpInfo.dpos;

	return 0;
}

// 弹出队首轨迹
int InterpBuffer::pop_front() {
	int idx = bufBeg % maxBufNum;

	// 预处理信息清空
	interpBuf[idx]->procInfo.reset();

	bufBeg++;
	return 0;
}

// 获取指定行号的轨迹类型
InterpSegmentType InterpBuffer::get_segment_type(int bufNum) {
	InterpSegmentType type = InterpSegmentType::JOINT;
	return type;
}

// 获取相邻的轨迹索引 [0,N)
int InterpBuffer::get_neighbor_index(int *pre, int *cur, int *next) {
	*next = (*cur + 1) % maxBufNum;
	*pre = (*cur - 1) < 0 ? maxBufNum - 1 : *cur - 1;
	*cur = *cur % maxBufNum;

	return 0;
}

// 执行轨迹插补
int InterpBuffer::move(PosData& pos) {
	int num = bufBeg, next, pre;
	get_neighbor_index(&pre, &num, &next);

	// 如果是新轨迹则进行规划
	if (interpBuf[num]->procInfo.processed && interpBuf[num]->procInfo.maxTime < dim_EPS) {
		interpBuf[num]->plan(*interpBuf[pre] , *interpBuf[next]);
	}

	// - 执行插补
	if (interpBuf[num]->motionCfg.moveType == 0) {
	}
	else if (interpBuf[num]->motionCfg.moveType == 0) {
	}
	interpBuf[num]->move(pos);

	// - 插补结果处理
	// 插补完成标识
	bool finish = interpBuf[num]->interpInfo.finish;

	return finish;
}



int InterpBuffer::joint_prehandle() {
	return 0;
}
int InterpBuffer::joint_plane() {
	return 0;
}
int InterpBuffer::joint_move() {
	return 0;
}

int InterpBuffer::cartesian_prehandle() {
	return 0;
}
int InterpBuffer::cartesian_plane() {
	return 0;
}
int InterpBuffer::cartesian_move() {
	return 0;
}
