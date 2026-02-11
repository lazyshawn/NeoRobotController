
#include "interpolation/InterpSegmentBuffer.h"

// 矩阵计算辅助库
#include "AuxMatrix.h"

static const double dim_EPS = 1e-6;

/***********************************************************************
 *                        InterpBuffer                                 *
 ***********************************************************************/
InterpBuffer::InterpBuffer() {
	bufBeg = bufEnd = 0;
	maxBufNum = 10;

	for (size_t i = 0; i < maxBufNum; ++i) {
		interpBuf.push_back(std::shared_ptr<InterpSegment>(new InterpSegment));
	}
}
InterpBuffer::~InterpBuffer() {

}

double InterpBuffer::get_cycleTime() {
	return cycleTime;
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
	traj = std::shared_ptr<InterpSegment>(new InterpSegment);
	//// 关节轨迹
	//if (cfg.moveType % 2 == 0) {
	//	traj = std::shared_ptr<InterpSegment>(new JointInterpSegment);
	//}
	//// 空间轨迹
	//else {
	//	traj = std::shared_ptr<InterpSegment>(new CartesianInterpSegment);
	//}

	// 轨迹加入缓冲
	traj->set_data(point, cfg, cmd);
	interpBuf[curBuf] = traj;

	// - 预处理
	//interpBuf[curBuf]->prehandle(*interpBuf[preBuf]);
	if (cfg.moveType % 2 == 0) {
		joint_prehandle();
	}
	else {
		cartesian_prehandle();
	}

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
// 获取相邻的轨迹指针
int InterpBuffer::get_neighbor_buffer(int cur, InterpSegment *&preBuf, InterpSegment *&curBuf, InterpSegment *&nextBuf) {
	int next = (cur + 1) % maxBufNum;
	int pre = (cur - 1) < 0 ? maxBufNum - 1 : cur - 1;
	cur = cur % maxBufNum;

	preBuf = interpBuf[pre].get();
	curBuf = interpBuf[cur].get();
	nextBuf = interpBuf[next].get();

	return 0;
}

// 执行轨迹插补
int InterpBuffer::move(PosData& pos) {
	int num = bufBeg, next, pre;
	get_neighbor_index(&pre, &num, &next);

	// 如果是新轨迹则进行规划
	if (interpBuf[num]->procInfo.processed && interpBuf[num]->procInfo.maxTime < dim_EPS) {
		if (interpBuf[num]->motionCfg.moveType == 0) {
			joint_plane();
		}
		else if (interpBuf[num]->motionCfg.moveType == 1) {
			cartesian_plan();
		}
	}

	// - 执行插补
	if (interpBuf[num]->motionCfg.moveType == 0) {
		joint_move();
	}
	else if (interpBuf[num]->motionCfg.moveType == 1) {
		cartesian_move();
	}

	// - 插补结果处理
	// 插补结果
	pos = interpBuf[num]->interpInfo.dpos;
	// 插补完成标识
	bool finish = interpBuf[num]->interpInfo.partId < 0;

	return finish;
}


int InterpBuffer::joint_prehandle() {
	// 预处理轨迹，队首开始
	InterpSegment *preBuf = nullptr, *curBuf = nullptr, *nextBuf = nullptr;
	get_neighbor_buffer(bufEnd, preBuf, curBuf, nextBuf);

	// - 把点位数据转化到关节空间

	// - 平滑处理
	// 当前段平滑设置
	curBuf->procInfo.postSmooth = 0;
	// 前段平滑设置
	if (preBuf->motionCfg.smooth > 0) {
		preBuf->procInfo.postSmooth = preBuf->motionCfg.smooth = curBuf->procInfo.preSmooth;
	}

	// - 预处理完毕
	curBuf->procInfo.processed = true;

	return 0;
}

int InterpBuffer::joint_plane() {
	// 规划轨迹，队首开始
	InterpSegment *preBuf = nullptr, *curBuf = nullptr, *nextBuf = nullptr;
	get_neighbor_buffer(bufBeg, preBuf, curBuf, nextBuf);

	// - 插补曲线规划
	// 当前段曲线规划
	curBuf->curve.set_condition(curBuf->pointInfo.begPos.rbtPos[0], curBuf->pointInfo.endPos.rbtPos[0], 0, 0);
	curBuf->curve.plan();

	double moveEndTime = 0.0;
	// 有后平滑，且下一段轨迹已插入
	if (curBuf->procInfo.postSmooth > 0 && nextBuf->procInfo.processed) {
		// 下一段曲线规划
		DoubleSCurve nextCurve;
		nextCurve.set_condition(nextBuf->pointInfo.begPos.rbtPos[0], nextBuf->pointInfo.endPos.rbtPos[0], 0, 0);
		nextCurve.plan();

		// 当前段结束时间即平滑开始时间
		double smoothTime = std::min(nextCurve.get_Ta(), curBuf->curve.get_Td()) * curBuf->motionCfg.smooth * 1e-2;
		moveEndTime = curBuf->curve.get_duration() - smoothTime;
	}
	else {
		moveEndTime = curBuf->curve.get_duration();
	}

	// 有前平滑，保存前段插补曲线
	if (curBuf->procInfo.preSmooth > 0) {
		curBuf->curvePre.set_condition(preBuf->pointInfo.begPos.rbtPos[0], preBuf->pointInfo.endPos.rbtPos[0], 0, 0);
		curBuf->curvePre.plan();
		curBuf->curvePre.displacement(preBuf->curTime - cycleTime, 1.0);
	}

	// 当前段规划时间
	curBuf->procInfo.maxTime = moveEndTime;

	// 插补时间从零计数
	curBuf->curTime = cycleTime;

	return 0;
}

int InterpBuffer::joint_move() {
	// 插补轨迹，队首开始
	InterpSegment *preBuf = nullptr, *curBuf = nullptr, *nextBuf = nullptr;
	get_neighbor_buffer(bufBeg, preBuf, curBuf, nextBuf);

	// 插补结果
	PosData pos;
	// 当前插补比例
	double ratio = curBuf->curTime / curBuf->procInfo.maxTime;

	pos.rbtPos = curBuf->pointInfo.begPos.rbtPos;
	// 有前平滑，前段曲线参与插补
	if (curBuf->procInfo.preSmooth > 0) {
		double prePos = curBuf->curvePre.get_pos(curBuf->curTime);
		double curPos = curBuf->curve.get_pos(curBuf->curTime);
		double curPos0 = curBuf->curve.get_pos(0);
		pos.rbtPos[0] = curBuf->curvePre.get_pos(curBuf->curTime) + curBuf->curve.get_pos(curBuf->curTime) - curBuf->curve.get_pos(0);
	}
	else {
		pos.rbtPos[0] = curBuf->curve.get_pos(curBuf->curTime);
	}

	// - 插补状态更新
	// 运动比例
	curBuf->interpInfo.schedule = ratio;
	// 当前目标位置
	curBuf->interpInfo.dpos = pos;

	// 插补时间累加
	curBuf->curTime += cycleTime;

	// 返回插补进度
	return 0;
}

int InterpBuffer::cartesian_prehandle() {
	// 预处理轨迹，队尾开始
	InterpSegment *preBuf = nullptr, *curBuf = nullptr, *nextBuf = nullptr;
	get_neighbor_buffer(bufEnd, preBuf, curBuf, nextBuf);

	// --- 把点位数据转化到笛卡尔空间

	// --- 当前段计算轨迹长度
	// 直线轨迹
	if ((curBuf->motionCfg.moveType & 2) == 0) {
		double det[3] = { 0 };
		for (int i = 0; i < 3; ++i) {
			det[i] = curBuf->pointInfo.endPos.rbtPos[i] - curBuf->pointInfo.begPos.rbtPos[i];
		}

		// 直线长度
		curBuf->procInfo.dist = std::sqrt(det[0] * det[0] + det[1] * det[1] + det[2] * det[2]);
		for (int i = 0; i < 3; ++i) {
			// 直线起点
			curBuf->procInfo.knot[i] = curBuf->pointInfo.begPos.rbtPos[i];
			// 直线方向
			curBuf->procInfo.dir[i] = det[i] / curBuf->procInfo.dist;
		}
	}
	// 圆弧轨迹
	else {

	}

	// --- 平滑处理
	// 当前段平滑设置
	curBuf->procInfo.postSmooth = 0;
	// 前段平滑设置
	if (preBuf->motionCfg.smooth > 0) {
		preBuf->procInfo.postSmooth = curBuf->procInfo.preSmooth = preBuf->motionCfg.smooth;
	}

	// --- 计算当前段前平滑控制点，前一段后平滑控制点
	if (curBuf->procInfo.preSmooth > 0) {
		double distPre = preBuf->procInfo.dist * 0.5;
		double distCur = curBuf->procInfo.dist * 0.5;
		// 平滑距离
		double smoothDist = std::min(distPre, distCur) * curBuf->procInfo.preSmooth * 1e-2;

		// 当前段前平滑
		curBuf->procInfo.preSmoothK = smoothDist / curBuf->procInfo.dist;
		// 前段后平滑
		preBuf->procInfo.postSmoothK = 1.0 - smoothDist / preBuf->procInfo.dist;

		// 直线拐点位置
		MatrixXd *corner = matrix_from_array(3, 1, curBuf->pointInfo.begPos.rbtPos.data(), 3);
		// 始末点切线方向
		MatrixXd *preDir = matrix_from_array(3, 1, preBuf->procInfo.dir, 3);
		MatrixXd *curDir = matrix_from_array(3, 1, curBuf->procInfo.dir, 3);
		// 计算当前段前平滑控制点
		MatrixXd *ctrl = matrix_copy(corner);
		double detK = 1.0 / 3;
		for (int i = 0; i < 3; ++i) {
			// 前半段 (u: 0 -> 0.5)
			matrix_plus(1, corner, -(1.0 - detK * i) * smoothDist, preDir, ctrl);
			matrix_to_array(ctrl, curBuf->procInfo.preCtrlPnt[i], 3);
			// 后半段 (u: 1 -> 0.5)
			matrix_plus(1, corner, (1.0 - detK * i) * smoothDist, curDir, ctrl);
			matrix_to_array(ctrl, curBuf->procInfo.preCtrlPnt[5 - i], 3);
		}
		// 前段后平滑控制点
		memcpy(preBuf->procInfo.postCtrlPnt, curBuf->procInfo.preCtrlPnt, 18 * sizeof(double));

		matrix_delete(preDir);
		matrix_delete(curDir);
		matrix_delete(ctrl);
	}

	// --- 计算规划段长度
	// 前平滑曲线长度 (前平滑曲线长度在前平滑曲线插补完后重新计算，因为会有一定的误差，影响过渡到直线的速度计算)
	curBuf->procInfo.preBlendDist = bezier_dist(5, curBuf->procInfo.preCtrlPnt, 0.5, 1, 1000);
	// 无平滑段长度
	curBuf->procInfo.mainDist = curBuf->procInfo.dist * (1.0 - curBuf->procInfo.preSmoothK);
	curBuf->procInfo.postBlendDist = 0.0;
	// 实际前平滑开始位置，在前平滑曲线插补完成后需要重新计算，使用实际运动距离，否则影响切换到直线时的速度
	curBuf->procInfo.segmBegDist = curBuf->procInfo.preBlendDist;
	curBuf->procInfo.segmEndDist = curBuf->procInfo.preBlendDist + curBuf->procInfo.mainDist;

	// 后平滑曲线长度
	preBuf->procInfo.postBlendDist = curBuf->procInfo.preBlendDist;
	preBuf->procInfo.mainDist = preBuf->procInfo.dist * (preBuf->procInfo.postSmoothK - preBuf->procInfo.preSmoothK);
	// 实际后平滑开始位置，前平滑曲线插补完成后同步偏移
	preBuf->procInfo.segmEndDist = preBuf->procInfo.preBlendDist + preBuf->procInfo.mainDist;

	// - 预处理完毕
	curBuf->procInfo.processed = true;

	return 0;
}

int InterpBuffer::cartesian_plan() {
	// 规划轨迹，队首开始
	InterpSegment *preBuf = nullptr, *curBuf = nullptr, *nextBuf = nullptr;
	get_neighbor_buffer(bufBeg, preBuf, curBuf, nextBuf);

	// --- 速度规划
	// 前瞻后溯
	
	// --- 插补曲线规划
	// 上一段剩余距离合并到前平滑段
	if (curBuf->procInfo.preSmooth > 0) {
		curBuf->procInfo.preBlendDist += preBuf->procInfo.remainS;
	}
	// 当前段规划总长度
	double planDist = curBuf->procInfo.preBlendDist + curBuf->procInfo.mainDist + curBuf->procInfo.postBlendDist;
	curBuf->curve.set_condition(0, planDist, 0, 0);
	curBuf->curve.plan();

	// 当前段规划时间
	curBuf->procInfo.maxTime = curBuf->curve.get_duration();

	// 计算整数倍插补周期后的剩余距离
	curBuf->procInfo.remainS = curBuf->curve.get_remain_dist(cycleTime);


	// --- 有前平滑时，获取上一段规划参数
	if (curBuf->procInfo.preSmooth > 0) {
		// 无平滑段始末位置
		curBuf->procInfo.segmBegDist = bezier_dist(5, curBuf->procInfo.preCtrlPnt, preBuf->interpInfo.doneU, 1.0, 1000);
		curBuf->procInfo.segmEndDist = curBuf->procInfo.segmBegDist + curBuf->procInfo.mainDist;

		// 位置分量不能合并插补，规划段右移，先插补上一条轨迹未完成的部分
		int shiftNum = ((preBuf->curve.get_duration() - preBuf->curve.get_offset()) - (preBuf->curTime - cycleTime)) / cycleTime;
		double shiftTime = shiftNum * cycleTime;
		// 右移整数个周期
		curBuf->curve.displacement(-shiftTime, 1);
		// 当前轨迹保留一个周期不插补
		curBuf->curve.set_reserve_time(cycleTime);

		// 前一条轨迹规划曲线左移: 前平滑大于零开始(curve.offset) -> 后平滑从零开始(curTime)
		preBuf->curve.displacement((preBuf->curTime - cycleTime) + preBuf->curve.get_offset(), 1);
		// 保留时间清零
		preBuf->curve.set_reserve_time(0);
	}

	// --- 插补状态复位
	// 插补完成标志复位
	curBuf->interpInfo.partId = 0;

	// 插补时间重新计数
	curBuf->curTime = cycleTime;

	return 0;
}

int InterpBuffer::cartesian_move() {
	// 插补轨迹，队首开始
	InterpSegment *preBuf = nullptr, *curBuf = nullptr, *nextBuf = nullptr;
	get_neighbor_buffer(bufBeg, preBuf, curBuf, nextBuf);
	// 插补结果
	PosData pos;
	
	// 上一段位移
	double preS = preBuf->interpInfo.doneS;
	bool preDone = true;
	if (curBuf->procInfo.preSmooth > 0) {
		preS = preBuf->curve.get_pos(curBuf->curTime);
		preDone = preBuf->curve.done();
	}
	// 当前时间位移
	double curS = curBuf->curve.get_pos(curBuf->curTime);
	bool curDone = curBuf->curve.done();

	double moveS = preS - preBuf->interpInfo.doneS;
	// 上一条轨迹插补完成
	if (preDone) {
		moveS += curS;
	}

	// 当前插补比例
	double ratio = curS / (curBuf->procInfo.preBlendDist + curBuf->procInfo.mainDist + curBuf->procInfo.postBlendDist);

	// 当前曲线参数，离开平滑段后复位
	double curU = 0.0;
	// 计算当前位置
	pos.rbtPos = curBuf->pointInfo.begPos.rbtPos;
	// 前平滑段
	if (curBuf->procInfo.preSmooth > 0 && moveS < curBuf->procInfo.segmBegDist) {
		if (curBuf->interpInfo.partId != 1) {
			curBuf->interpInfo.partId = 1;
		}
		double curPos[3];
		curU = bezier_interp(5, curBuf->procInfo.preCtrlPnt, curBuf->interpInfo.curU, moveS - curBuf->interpInfo.curS);
		bezier_positioin(5, curBuf->procInfo.preCtrlPnt, curU, curPos);

		for (int i = 0; i < 3; ++i) {
			pos.rbtPos[i] = curPos[i];
		}
	}
	else {
		// 前平滑段结束后，基于实际走的S和曲线计算的S偏差重新计算 `前平滑段长度`，这样切换到直线段时速度计算才准确
		if (curBuf->interpInfo.partId == 1 && curBuf->procInfo.preSmooth > 0) {
			double realDist = bezier_dist(5, curBuf->procInfo.preCtrlPnt, preBuf->interpInfo.doneU, curBuf->interpInfo.curU, 1000);
			double error = curBuf->interpInfo.curS - realDist;
			double maxError = moveS - curBuf->procInfo.segmBegDist;
			// 避免直线段计算的距离为负数
			// 实际直线计算是从距离大于curS开始的，计算的曲线距离(realDist) -> 实际规划距离(interpInfo.curS)
			curBuf->procInfo.segmBegDist += error;
			curBuf->procInfo.segmEndDist += error;
		}

		// 后平滑段
		if (curBuf->procInfo.postSmooth > 0 && moveS > curBuf->procInfo.segmEndDist) {
			// 后平滑段上的位移增量
			double dis = moveS - curBuf->interpInfo.curS;

			if (curBuf->interpInfo.partId != 3) {
				curBuf->interpInfo.partId = 3;
				curBuf->interpInfo.curU = 0;
				dis = moveS - curBuf->procInfo.segmBegDist - curBuf->procInfo.mainDist;
			}

			double curPos[3];
			curU = bezier_interp(5, curBuf->procInfo.postCtrlPnt, curBuf->interpInfo.curU, dis);
			bezier_positioin(5, curBuf->procInfo.postCtrlPnt, curU, curPos);

			for (int i = 0; i < 3; ++i) {
				pos.rbtPos[i] = curPos[i];
			}

			// 后平滑输出第一个点后结束当前段插补，剩下的部分再下一条轨迹中插补
			curDone = true;
		}
		// 无平滑段
		else {
			if (curBuf->interpInfo.partId != 2) {
				curBuf->interpInfo.partId = 2;
			}

			double dis = moveS - curBuf->procInfo.segmBegDist + curBuf->procInfo.preSmoothK * curBuf->procInfo.dist;
			for (int i = 0; i < 3; ++i) {
				pos.rbtPos[i] = curBuf->pointInfo.begPos.rbtPos[i] * (1.0 - dis / curBuf->procInfo.dist) + curBuf->pointInfo.endPos.rbtPos[i] * dis / curBuf->procInfo.dist;
			}
		}
	}

	// - 插补状态更新
	// 插补进度
	curBuf->interpInfo.schedule = ratio;
	// 当前目标位置
	curBuf->interpInfo.dpos = pos;
	curBuf->interpInfo.curS = moveS;
	curBuf->interpInfo.curU = curU;

	// 插补完成: 有平滑时，剩余规划时间小于一个插补周期，将剩余距离移到下条轨迹前平滑段内插补
	if (curDone) {
		curBuf->interpInfo.partId = -1;
		// 下一段平滑从当前后平滑继续
		nextBuf->interpInfo.curU = curBuf->interpInfo.curU;
		curBuf->interpInfo.doneS = curS;
		curBuf->interpInfo.doneU = curU;
	}

	// 插补时间累加
	curBuf->curTime += cycleTime;

	return 0;
}
