
#include "InterpSegmentBuffer.h"

// 矩阵计算辅助库
#include "AuxMatrix.h"
#include "InterpCurve.h"

static const double dim_EPS = 1e-6;


/***********************************************************************
 *                        A U X I L I A R Y                            *
 ***********************************************************************/
// 计算轨迹参数
int calc_traj_info(const double *begPnt, const double *midPnt, const double *endPnt, int mode, double *ans) {
	// 预声明所有资源并初始化为NULL
	MatrixXd *begPos = matrix_new(3, 1, 0), *midPos = matrix_new(3, 1, 0), *endPos = matrix_new(3, 1, 0);
	MatrixXd *a = matrix_new(3, 1, 0), *b = matrix_new(3, 1, 0), *aXb = matrix_new(3, 1, 0);
	MatrixXd *tmp = matrix_new(3, 1, 0), *cent = matrix_new(3, 1, 0);
	MatrixXd *op1 = matrix_new(3, 1, 0), *op2 = matrix_new(3, 1, 0), *op3 = matrix_new(3, 1, 0);
	MatrixXd *n12 = matrix_new(3, 1, 0), *n13 = matrix_new(3, 1, 0), *normal = matrix_new(3, 1, 0);
	MatrixXd *radius = matrix_new(3, 1, 0), *lineDir = matrix_new(3, 1, 0);

	// 直线拐点位置
	begPos = matrix_from_array(3, 1, begPnt, 3);
	midPos = matrix_from_array(3, 1, midPnt, 3);
	endPos = matrix_from_array(3, 1, endPnt, 3);

	double knot[3], dir[3], dist;
	int isLine = (mode == 0) ? 1 : 0;

	if (mode == 1) {
		a = matrix_new(3, 1, 0);
		b = matrix_new(3, 1, 0);
		matrix_plus(1.0, begPos, -1.0, midPos, a);
		matrix_plus(1.0, endPos, -1.0, midPos, b);
		// 当直线处理
		//if (a.cross(b).squaredNorm() < 1e-9) {
		//	info.head(3) = (endPos - begPos).normalized();
		//	info[3] = (endPos - begPos).norm();
		//	isLine = true;
		//}

		// 圆心位置
		aXb = matrix_new(3, 1, 0);
		matrix_outer_product(a, b, aXb);
		double La = matrix_norm(a), Lb = matrix_norm(b), LaXb = matrix_norm(aXb);
		tmp = matrix_new(3, 1, 0);
		cent = matrix_new(3, 1, 0);
		matrix_plus(La*La / (2*LaXb*LaXb), b, -Lb*Lb / (2 * LaXb*LaXb), a, tmp);
		matrix_outer_product(tmp, aXb, cent);
		matrix_plus(1, cent, 1, midPos, cent);

		// 半径方向
		op1 = matrix_new(3, 1, 0);
		op2 = matrix_new(3, 1, 0);
		op3 = matrix_new(3, 1, 0);

		// 计算半径方向向量
		matrix_plus(1.0, begPos, -1.0, cent, op1);
		matrix_plus(1.0, midPos, -1.0, cent, op2);
		matrix_plus(1.0, endPos, -1.0, cent, op3);
		// 单位化
		matrix_normalize(op1);
		matrix_normalize(op2);
		matrix_normalize(op3);

		// 法线方向
		n12 = matrix_new(3, 1, 0);
		n13 = matrix_new(3, 1, 0);
		matrix_outer_product(op1, op2, n12);
		matrix_outer_product(op1, op3, n13);

		// 圆弧运动平面的法线方向
		normal = matrix_new(3, 1, 0);
		matrix_outer_product(op1, op3, normal);

		// 半径夹角
		double q12 = acos(matrix_inner_product(op1, op2));
		double q13 = acos(matrix_inner_product(op1, op3));

		// 圆心角度
		double theta = acos(matrix_inner_product(op1, op3));

		// 修正圆心角和法向量
		// 2,3 在 1 的两侧
		if (matrix_inner_product(n12, n13) < 0) {
			matrix_scale(normal, -1.0);
			theta = 2 * M_PI - theta;
		}
		// q12 > q13
		else if (q12 > q13) {
			matrix_scale(normal, -1.0);
			theta = 2 * M_PI - theta;
		}
		else if (q13 > q12) {
			matrix_delete(normal);
			normal = matrix_new_copy(n12);
		}

		// 单位化法线方向并乘以圆心角度
		matrix_normalize(normal);
		matrix_scale(normal, theta);

		// 计算圆弧长度
		radius = matrix_new(3, 1, 0);
		matrix_plus(1.0, begPos, -1.0, cent, radius);
		double radius_len = matrix_norm(radius);
		dist = radius_len * theta;

		// 将cent的值赋给knot数组
		matrix_to_array(cent, knot, 3);
		// 将normal的值赋给dir数组
		matrix_to_array(normal, dir, 3);
	}

	if (isLine) {
		// 计算直线方向向量
		matrix_plus(1.0, endPos, -1.0, begPos, lineDir);
		// 计算直线长度
		dist = matrix_norm(lineDir);
		// 单位化直线方向向量
		matrix_normalize(lineDir);

		// 计算轨迹信息
		matrix_to_array(begPos, knot, 3);
		matrix_to_array(lineDir, dir, 3);
	}

	for (int i = 0; i < 3; ++i) {
		ans[i] = knot[i];
		ans[i + 4] = dir[i];
	}
	ans[3] = dist;

	// 集中释放所有资源
	matrix_delete(begPos);
	matrix_delete(midPos);
	matrix_delete(endPos);
	matrix_delete(a);
	matrix_delete(b);
	matrix_delete(aXb);
	matrix_delete(tmp);
	matrix_delete(cent);
	matrix_delete(op1);
	matrix_delete(op2);
	matrix_delete(op3);
	matrix_delete(n12);
	matrix_delete(n13);
	matrix_delete(normal);
	matrix_delete(radius);
	matrix_delete(lineDir);

	return 0;
}

// 计算平滑控制点
int calc_smooth_ctrl_pnt(const InterpSegment *preBuf, const InterpSegment *curBuf, double smoothDist, double ctrlPnt[6][3]) {

	MatrixXd *preDir = matrix_from_array(3, 1, preBuf->procInfo.dir, 3), *prePos = matrix_new(3, 1, 0);
	MatrixXd *curDir = matrix_from_array(3, 1, curBuf->procInfo.dir, 3), *curPos = matrix_new(3, 1, 0);
	MatrixXd *corner = matrix_from_array(3, 1, curBuf->pointInfo.begPos.rbtPos.data(), 3);
	MatrixXd *ctrl = matrix_new_copy(corner);

	// 前段轨迹平滑起点和切向量
	if (preBuf->motionCfg.moveType == 1) {
		matrix_plus(1.0, corner, -smoothDist, preDir, prePos);
	}
	else if (preBuf->motionCfg.moveType == 2) {
		// 终点半径向量
		MatrixXd *radius = matrix_new(3, 1, 0), *center = matrix_from_array(3, 1, preBuf->procInfo.knot, 3);
		MatrixXd *rotDir = matrix_from_array(3, 1, preBuf->procInfo.dir, 3), *uxv = matrix_new(3, 1, 0);
		MatrixXd *tanDir = matrix_new(3, 1, 0);
		matrix_plus(1.0, corner, -1.0, center, radius);
		matrix_normalize(rotDir);

		// 旋转后的半径向量
		double theta = - smoothDist / matrix_norm(radius), cq = cos(theta), sq = sin(theta);
		// 平滑开始点
		matrix_outer_product(rotDir, radius, uxv);
		matrix_plus(cq, radius, (1.0 - cq) * matrix_inner_product(rotDir, radius), rotDir, prePos);
		matrix_plus(1.0, prePos, sq, uxv, prePos);
		matrix_plus(1.0, prePos, 1.0, center, prePos);

		// 切向量
		tanDir = matrix_new_copy(uxv);
		matrix_outer_product(rotDir, tanDir, uxv);
		matrix_plus(cq, tanDir, (1.0 - cq) * matrix_inner_product(rotDir, tanDir), rotDir, preDir);
		matrix_plus(1.0, preDir, sq, uxv, preDir);
		matrix_normalize(preDir);

		matrix_delete(radius);
		matrix_delete(center);
		matrix_delete(rotDir);
		matrix_delete(tanDir);
		matrix_delete(uxv);
	}

	// 当前轨迹平滑终点和切向量
	if (curBuf->motionCfg.moveType == 1) {
		matrix_plus(1.0, corner, smoothDist, curDir, curPos);
	}
	else if (curBuf->motionCfg.moveType == 2) {
		// 终点半径向量
		MatrixXd *radius = matrix_new(3, 1, 0), *center = matrix_from_array(3, 1, curBuf->procInfo.knot, 3);
		MatrixXd *rotDir = matrix_from_array(3, 1, curBuf->procInfo.dir, 3), *uxv = matrix_new(3, 1, 0);
		MatrixXd *tanDir = matrix_new(3, 1, 0);
		matrix_plus(1.0, corner, -1.0, center, radius);
		matrix_normalize(rotDir);

		// 旋转后的半径向量
		double theta = smoothDist / matrix_norm(radius), cq = cos(theta), sq = sin(theta);
		// 平滑结束点
		matrix_outer_product(rotDir, radius, uxv);
		matrix_plus(cq, radius, (1.0 - cq) * matrix_inner_product(rotDir, radius), rotDir, curPos);
		matrix_plus(1.0, curPos, sq, uxv, curPos);
		matrix_plus(1.0, curPos, 1.0, center, curPos);

		// 切向量
		tanDir = matrix_new_copy(uxv);
		matrix_outer_product(rotDir, tanDir, uxv);
		matrix_plus(cq, tanDir, (1.0 - cq) * matrix_inner_product(rotDir, tanDir), rotDir, curDir);
		matrix_plus(1.0, curDir, sq, uxv, curDir);
		matrix_normalize(curDir);

		matrix_delete(radius);
		matrix_delete(center);
		matrix_delete(rotDir);
		matrix_delete(tanDir);
		matrix_delete(uxv);
	}

	double detK = 1.0 / 3;
	for (int i = 0; i < 3; ++i) {
		// 前半段 (u: 0 -> 0.5)
		matrix_plus(1.0, prePos, detK * i * smoothDist, preDir, ctrl);
		matrix_to_array(ctrl, ctrlPnt[i], 3);
		// 后半段 (u: 1 -> 0.5)
		matrix_plus(1.0, curPos, -detK * i * smoothDist, curDir, ctrl);
		matrix_to_array(ctrl, ctrlPnt[5 - i], 3);
	}

	matrix_delete(preDir);
	matrix_delete(prePos);
	matrix_delete(curDir);
	matrix_delete(curPos);
	matrix_delete(corner);
	matrix_delete(ctrl);
	return 0;
}

// 计算空间轨迹比例分割点
int calc_cartesian_breakpoint(const InterpSegment *curBuf, double lambda, int mode, double *dpos) {
	// 直线
	if (mode == 0) {
		for (int i = 0; i < 3; ++i) {
			dpos[i] = curBuf->pointInfo.begPos.rbtPos[i] * (1.0 - lambda) + curBuf->pointInfo.endPos.rbtPos[i] * lambda;
		}
	}
	// 圆弧
	else if (mode == 1) {
		// 终点半径向量
		MatrixXd *corner = matrix_from_array(3, 1, curBuf->pointInfo.begPos.rbtPos.data(), 3);
		MatrixXd *radius = matrix_new(3, 1, 0), *center = matrix_from_array(3, 1, curBuf->procInfo.knot, 3);
		MatrixXd *rotDir = matrix_from_array(3, 1, curBuf->procInfo.dir, 3), *uxv = matrix_new(3, 1, 0);
		matrix_plus(1.0, corner, -1.0, center, radius);
		matrix_normalize(rotDir);

		// 旋转后的半径向量
		double theta = lambda * curBuf->procInfo.dist / matrix_norm(radius), cq = cos(theta), sq = sin(theta);
		matrix_outer_product(rotDir, radius, uxv);
		matrix_plus(cq, radius, (1.0 - cq) * matrix_inner_product(rotDir, radius), rotDir, corner);
		matrix_plus(1.0, corner, sq, uxv, corner);

		// 平滑结束
		matrix_plus(1.0, corner, 1.0, center, corner);
		for (int i = 0; i < 3; ++i) {
			dpos[i] = corner->data[i];
		}

		matrix_delete(corner);
		matrix_delete(radius);
		matrix_delete(center);
		matrix_delete(rotDir);
		matrix_delete(uxv);
	}

	return 0;
}

/***********************************************************************
 *                        InterpBuffer                                 *
 ***********************************************************************/
InterpBuffer::InterpBuffer() {
	bufBeg = bufEnd = 0;
	maxBufNum = 10;
	// 保留一个缓冲位置，用于记录上一条运动
	reserveNum = 1;

	for (size_t i = 0; i < maxBufNum; ++i) {
		interpBuf.push_back(std::shared_ptr<InterpSegment>(new InterpSegment()));
	}
}

InterpBuffer::~InterpBuffer() {

}

double InterpBuffer::get_cycleTime() {
	return cycleTime;
}

bool InterpBuffer::buffer_ready() {
	return (!bufOccupied) && (bufEnd - bufBeg - maxBufNum + reserveNum < 0);
}

int InterpBuffer::add_move_point(const PointInfo& point, const MotionCfg& cfg, const MoveCmd& cmd) {
	// - 轨迹队列已满
	if (bufEnd - bufBeg - maxBufNum + reserveNum == 0) {
		printf("point buffer is full: %d\n", maxBufNum);
		return 1;
	}

	// 轨迹写入位置
	int curBuf = bufEnd, nextBuf, preBuf;
	get_neighbor_index(&preBuf, &curBuf, &nextBuf);

	// 写入位置行号为非负，表示点位未执行完毕，不插入点位
	if (interpBuf[curBuf]->procInfo.lineNum >= 0) {
		return -1;
	}

	// - 插入轨迹
	bufOccupied = true;
	// 缓冲轨迹
	std::shared_ptr<InterpSegment> traj = std::make_shared<InterpSegment>();

	// 轨迹加入缓冲
	traj->set_data(point, cfg, cmd);
	interpBuf[curBuf] = traj;

	// - 预处理
	interpBuf[curBuf]->procInfo.procStage = 1;
	if (cfg.moveType == 0)
		joint_prehandle();
	else if(cfg.moveType == 1 || cfg.moveType == 2)
		cartesian_prehandle();
	interpBuf[curBuf]->procInfo.procStage = 2;

	// 行号
	interpBuf[curBuf]->procInfo.lineNum = bufEnd;

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
	*pre = (*cur - 1) < 0 ? maxBufNum - 1 : (*cur - 1) % maxBufNum;
	*cur = *cur % maxBufNum;

	return 0;
}
// 获取相邻的轨迹指针
int InterpBuffer::get_neighbor_buffer(int cur, InterpSegment *&preBuf, InterpSegment *&curBuf, InterpSegment *&nextBuf) {
	int next = (cur + 1) % maxBufNum;
	int pre = (cur - 1) < 0 ? maxBufNum - 1 : (cur - 1) % maxBufNum;
	cur = cur % maxBufNum;

	preBuf = interpBuf[pre].get();
	curBuf = interpBuf[cur].get();
	nextBuf = interpBuf[next].get();

	return 0;
}

InterpSegment *InterpBuffer::get_following_buffer(int cur, int accent) {
	cur = (cur + accent) % maxBufNum;

	return interpBuf[cur].get();
}

// 执行轨迹插补
int InterpBuffer::move(PosData& pos) {
	int num = bufBeg, next, pre;
	get_neighbor_index(&pre, &num, &next);

	// 如果是第一条轨迹，等待若干周期，保证前两条轨迹平滑生效
	// 如果是新轨迹则进行规划, 预处理完成，未开始规划
	if (interpBuf[num]->procInfo.procStage == 2) {
		interpBuf[num]->procInfo.procStage = 3;
		if (interpBuf[num]->motionCfg.moveType == 0)
			joint_plane();
		else if (interpBuf[num]->motionCfg.moveType > 0)
			cartesian_plan();
		interpBuf[num]->procInfo.procStage = 4;
	}

	// - 执行插补
	if (interpBuf[num]->motionCfg.moveType == 0) {
		joint_move();
	}
	else if (interpBuf[num]->motionCfg.moveType == 1 || interpBuf[num]->motionCfg.moveType == 2) {
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
	// 当前段后平滑置零，默认停在当前点
	curBuf->procInfo.postSmooth = 0;
	// 前段平滑设置
	if (preBuf->motionCfg.smooth > 0) {
		preBuf->procInfo.postSmooth = curBuf->procInfo.preSmooth = preBuf->motionCfg.smooth;
	}

	return 0;
}

int InterpBuffer::joint_plane() {
	// 规划轨迹，队首开始
	InterpSegment *preBuf = nullptr, *curBuf = nullptr, *nextBuf = nullptr;
	get_neighbor_buffer(bufBeg, preBuf, curBuf, nextBuf);

	// --- 1. 有前平滑，保存前段插补曲线
	if (curBuf->procInfo.preSmooth > 0) {
		for (int i = 0; i < 9; ++i) {
			curvePre[i] = curve[i];
			// 上一段未插补的部分移动到时间起点，保留缩放
			curvePre[i].displacement(-(preBuf->procInfo.curTime - cycleTime), curve[i].get_scale());
			curve[i].clear();
		}
	}

	// --- 2. 当前段曲线规划
	double curDuration = 0.0, curTd = 0.0;
	for (int i = 0; i < 9; ++i) {
		double beg = i < 6 ? curBuf->pointInfo.begPos.rbtPos[i] : curBuf->pointInfo.begPos.extPos[i - 6];
		double end = i < 6 ? curBuf->pointInfo.endPos.rbtPos[i] : curBuf->pointInfo.endPos.extPos[i - 6];
		curve[i].set_condition(beg, end, 0, 0);
		curve[i].plan();
		if (curve[i].get_duration() > curDuration) {
			curTd = curve[i].get_Td();
			curDuration = curve[i].get_duration();
		}
	}
	// 当前段规划同步
	if (curDuration > dim_EPS) {
		for (int i = 0; i < 9; ++i) {
			curve[i].displacement(0, curDuration / curve[i].get_duration());
		}
	}

	// --- 3. 计入平滑后当前段的插补时长
	double moveEndTime = curDuration;
	// 有后平滑，且下一段轨迹已经插入
	if (curBuf->procInfo.postSmooth > 0 && nextBuf->procInfo.procStage == 2) {
		// 计算下一段轨迹规划出的总时长
		double nextDuration = 0.0, nextTa = 0.0;
		DoubleSCurve nextCurve;
		for (int i = 0; i < 9; ++i) {
			double beg = i < 6 ? nextBuf->pointInfo.begPos.rbtPos[i] : nextBuf->pointInfo.begPos.extPos[i - 6];
			double end = i < 6 ? nextBuf->pointInfo.endPos.rbtPos[i] : nextBuf->pointInfo.endPos.extPos[i - 6];
			nextCurve.clear();
			nextCurve.set_condition(beg, end, 0, 0);
			nextCurve.plan();
			if (nextCurve.get_duration() > nextDuration) {
				nextTa = curve[i].get_Ta();
				nextDuration = nextCurve.get_duration();
			}
		}
		double smoothTime = std::min(curTd, nextTa) * curBuf->motionCfg.smooth * 1e-2;
		moveEndTime = curDuration - smoothTime;
	}
	// 保存当前段实际插补时长
	curBuf->procInfo.maxTime = moveEndTime;

	// 插补时间从零计数
	curBuf->procInfo.curTime = cycleTime;

	return 0;
}

int InterpBuffer::joint_move() {
	// 插补轨迹，队首开始
	InterpSegment *preBuf = nullptr, *curBuf = nullptr, *nextBuf = nullptr;
	get_neighbor_buffer(bufBeg, preBuf, curBuf, nextBuf);

	// 插补结果
	PosData pos = curBuf->pointInfo.begPos;
	// 当前插补比例
	double ratio = curBuf->procInfo.curTime / curBuf->procInfo.maxTime;

	std::vector<double> posVec(9, 0.0);
	for (int i = 0; i < 9; ++i) {
		posVec[i] = curve[i].get_pos(curBuf->procInfo.curTime);
	}
	// 有前平滑，前段曲线参与插补
	if (curBuf->procInfo.preSmooth > 0) {
		for (int i = 0; i < 9; ++i) {
			double tmp = curvePre[i].get_pos(curBuf->procInfo.curTime);
			posVec[i] += curvePre[i].get_pos(curBuf->procInfo.curTime) - curve[i].get_pos(0);
		}
	}
	for (int i = 0; i < 6; ++i) {
		pos.rbtPos[i] = posVec[i];
	}
	for (int i = 6; i < 9; ++i) {
		pos.extPos[i - 6] = posVec[i];
	}

	// - 插补状态更新
	// 运动比例
	curBuf->interpInfo.schedule = ratio;
	// 当前目标位置
	curBuf->interpInfo.dpos = pos;

	// 插补时间累加
	curBuf->procInfo.curTime += cycleTime;
	// 当前段插补结束
	if (curBuf->procInfo.curTime > curBuf->procInfo.maxTime) {
		curBuf->interpInfo.partId = -1;
	}

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
	if (curBuf->motionCfg.moveType == 1) {
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
	else if (curBuf->motionCfg.moveType == 2) {
		double det[7] = { 0 };
		calc_traj_info(curBuf->pointInfo.begPos.rbtPos.data(), curBuf->pointInfo.midPos.rbtPos.data(), curBuf->pointInfo.endPos.rbtPos.data(), 1, det);

		for (int i = 0; i < 3; ++i) {
			curBuf->procInfo.knot[i] = det[i];
			curBuf->procInfo.dir[i] = det[i + 4];
		}
		curBuf->procInfo.dist = det[3];
	}

	// --- 平滑处理
	// 当前段平滑设置
	curBuf->procInfo.postSmooth = 0;
	// 前段平滑设置，前段未开始规划，可能需要提前到前两条轨迹结束前若干个周期，以免当前段预处理时前一段正好开始规划
	if (preBuf->motionCfg.smooth > 0 && preBuf->procInfo.procStage == 2) {
		preBuf->procInfo.postSmooth = curBuf->procInfo.preSmooth = preBuf->motionCfg.smooth;
	}

	// --- 计算当前段前平滑控制点，前一段后平滑控制点
	if (curBuf->procInfo.preSmooth > 0) {
		double distPre = preBuf->procInfo.dist * 0.5;
		double distCur = curBuf->procInfo.dist * 0.5;
		// 平滑距离
		double smoothDist = std::min(distPre, distCur) * curBuf->procInfo.preSmooth * 1e-2;
		// 重复点，平滑距离为0，取消平滑
		if (smoothDist < dim_EPS) {
			curBuf->procInfo.preSmooth = 0.0;
			preBuf->procInfo.postSmooth = 0.0;

			curBuf->procInfo.preSmoothK = 0.0;
			preBuf->procInfo.postSmoothK = 1.0;
		}
		else {
			// 当前段前平滑
			curBuf->procInfo.preSmoothK = smoothDist / curBuf->procInfo.dist;
			// 前段后平滑
			preBuf->procInfo.postSmoothK = 1.0 - smoothDist / preBuf->procInfo.dist;

			calc_smooth_ctrl_pnt(preBuf, curBuf, smoothDist, curBuf->procInfo.preCtrlPnt);

			// 前段后平滑控制点
			memcpy(preBuf->procInfo.postCtrlPnt, curBuf->procInfo.preCtrlPnt, 18 * sizeof(double));
		}
	}

	// --- 计算规划段长度
	if (curBuf->procInfo.preSmooth > 0) {
		// 前平滑曲线长度 (前平滑曲线长度在前平滑曲线插补完后重新计算，因为会有一定的误差，影响过渡到直线的速度计算)
		curBuf->procInfo.preBlendDist = bezier_dist(5, curBuf->procInfo.preCtrlPnt, 0.5, 1, 1000);
	}
	else {
		curBuf->procInfo.preBlendDist = 0.0;
	}
	// 无平滑段长度
	curBuf->procInfo.mainDist = curBuf->procInfo.dist * (1.0 - curBuf->procInfo.preSmoothK);
	curBuf->procInfo.postBlendDist = 0.0;
	// 实际前平滑开始位置，在前平滑曲线插补完成后需要重新计算，使用实际运动距离，否则影响切换到直线时的速度
	curBuf->procInfo.segmBegDist = curBuf->procInfo.preBlendDist;
	curBuf->procInfo.segmEndDist = curBuf->procInfo.preBlendDist + curBuf->procInfo.mainDist;

	// --- 前一段曲线参数, 预处理完成，未开始规划
	if (preBuf->procInfo.procStage == 2) {
		// 后平滑曲线长度
		preBuf->procInfo.postBlendDist = curBuf->procInfo.preBlendDist;
		preBuf->procInfo.mainDist = preBuf->procInfo.dist * (preBuf->procInfo.postSmoothK - preBuf->procInfo.preSmoothK);
		// 实际后平滑开始位置，前平滑曲线插补完成后同步偏移
		preBuf->procInfo.segmEndDist = preBuf->procInfo.preBlendDist + preBuf->procInfo.mainDist;

		// 前一段轨迹的终点约束速度
		preBuf->procInfo.constrainedVel = std::min(preBuf->motionCfg.speed, curBuf->motionCfg.speed);
	}

	return 0;
}

int InterpBuffer::cartesian_plan() {
	// 规划轨迹，队首开始
	InterpSegment *preBuf = nullptr, *curBuf = nullptr, *nextBuf = nullptr;
	get_neighbor_buffer(bufBeg, preBuf, curBuf, nextBuf);

	// 插补曲线更新
	for (int i = 0; i < 9; ++i) {
		curvePre[i] = curve[i];
		curve[i].clear();
	}

	// --- 速度规划
	// 设置位置规划约束
	curve[0].set_constraint(curBuf->motionCfg.speed, 10, 10*10);
	double vs = preBuf->procInfo.constrainedVel;
	// 前瞻回溯
	double ve = cartesian_look_ahead();
	curBuf->procInfo.constrainedVel = ve;
	
	// --- 插补曲线规划
	// 当前段规划总长度
	double planDist = curBuf->procInfo.preBlendDist + curBuf->procInfo.mainDist + curBuf->procInfo.postBlendDist;
	// 上一段剩余距离合并到前平滑段
	if (curBuf->procInfo.preSmooth > 0) {
		planDist += preBuf->procInfo.remainS;
	}
	curve[0].set_condition(0, planDist, vs, ve);
	curve[0].plan();

	// 计算整数倍插补周期后的剩余距离
	curBuf->procInfo.remainS = curve[0].get_remain_dist(cycleTime);
	curBuf->procInfo.remainT = curve[0].get_remain_time(cycleTime);

	// 当前段规划时间
	curBuf->procInfo.maxTime = curve[0].get_duration();

	// --- 有前平滑时，获取上一段规划参数
	if (curBuf->procInfo.preSmooth > 0) {
		// 无平滑段始末位置
		curBuf->procInfo.segmBegDist = bezier_dist(5, curBuf->procInfo.preCtrlPnt, preBuf->procInfo.doneU, 1.0, 1000);
		curBuf->procInfo.segmEndDist = curBuf->procInfo.segmBegDist + curBuf->procInfo.mainDist;

		// 位置分量不能合并插补，规划段右移，先插补上一条轨迹未完成的部分
		int shiftNum = ((curvePre[0].get_duration() + curvePre[0].get_offset()) - (preBuf->procInfo.curTime - cycleTime)) / cycleTime;
		double shiftTime = shiftNum * cycleTime;
		// 右移整数个周期
		curve[0].displacement(shiftTime, 1);

		// 前一条轨迹规划曲线左移: 前平滑大于零开始(curve.offset) -> 后平滑从零开始(curTime)
		curvePre[0].displacement(-(preBuf->procInfo.curTime - cycleTime) + curvePre[0].get_offset(), 1);

		// 前一条轨迹附加轴规划左移
		curvePre[6].displacement(-(preBuf->procInfo.curTime - cycleTime), 1);
	}
	else {
		// 主运动状态初始化
		interpStatus.vel = 0.0;
		for (int i = 0; i < 3; ++i) {
			interpStatus.tan[i] = 0.0;
			interpStatus.cPos[i] = curBuf->pointInfo.begPos.rbtPos[i];
		}
	}

	// 有后平滑
	if (curBuf->procInfo.postSmooth > 0) {
		// 当前轨迹保留不足一个周期的部分，在下一条轨迹内插补
		curve[0].set_reserve_time(curBuf->procInfo.remainT);
	}

	// --- 附加轴规划
	curve[6].set_condition(curBuf->pointInfo.begPos.extPos[0], curBuf->pointInfo.endPos.extPos[0], 0, 0);
	double time1 = curve[0].get_offset();
	double time2 = curve[0].calc_time_PiTPe(curBuf->procInfo.postBlendDist);
	// 附加轴规划段的位移时间
	double eulerTime = curve[0].get_duration() + curve[0].get_offset();
	// 姿态加速时间
	double oriAccT = 0.0;
	// 无平滑: 姿态匀速时长 = 位置匀速时长
	if (curBuf->procInfo.preSmooth < dim_EPS && curBuf->procInfo.postSmooth < dim_EPS)
		oriAccT = (curve[0].get_duration() - curve[0].get_Tv()) / 2;
	// 有前后平滑
	else if (curBuf->procInfo.preSmooth > dim_EPS && curBuf->procInfo.postSmooth > dim_EPS)
		oriAccT = time1;
	// 只有前平滑
	else if (curBuf->procInfo.preSmooth > dim_EPS && curBuf->procInfo.postSmooth < dim_EPS)
		oriAccT = 1.5 * time1;
	// 只有后平滑
	else if (curBuf->procInfo.preSmooth < dim_EPS && curBuf->procInfo.postSmooth > dim_EPS)
		oriAccT = 1.5 * time2;
	curve[6].plan_by_duration(eulerTime, oriAccT, oriAccT / 2);

	// --- 摆焊规划
	SwingConfig swing = curBuf->motionCfg.swingParam;
	// 有前平滑时，继承前段的规划，
	if (curBuf->procInfo.preSmooth > 0) {
		// --- 继承摆焊参数
		curve[2] = curvePre[2];

		// 缓存下条轨迹的摆焊参数
		rtMotionCfgBuf.swingParam = nextBuf->motionCfg.swingParam;
	}
	else {
		swingProc.state = 0;
		swingProc.time = 0.0;
		swingProc.pos = 0.0;
	}
	// 后平滑决定摆焊运动时间
	if (curBuf->procInfo.postSmooth > 0) {
		swingProc.duration = 2 * (curve[0].get_duration() + curve[0].get_offset());
	}
	else {
		swingProc.duration = curve[0].get_duration() + curve[0].get_offset();
	}
	rtMotionCfg.swingParam = swing;

	// --- 插补状态复位
	// 插补完成标志复位
	curBuf->interpInfo.partId = 0;

	// 插补时间重新计数
	curBuf->procInfo.curTime = cycleTime;

	return 0;
}

int InterpBuffer::cartesian_move() {
	// 插补轨迹，队首开始
	InterpSegment *preBuf = nullptr, *curBuf = nullptr, *nextBuf = nullptr;
	get_neighbor_buffer(bufBeg, preBuf, curBuf, nextBuf);
	// 插补结果
	PosData pos = curBuf->pointInfo.begPos;
	
	// --- 计算上一段+当前段的总位移
	// 上一段规划位移
	double preS = preBuf->procInfo.doneS;
	bool preDone = true;
	if (curBuf->procInfo.preSmooth > 0) {
		preS = curvePre[0].get_pos(curBuf->procInfo.curTime);
		preDone = curvePre[0].done();
	}
	// 当前段规划位移
	double curS = curve[0].get_pos(curBuf->procInfo.curTime);
	bool curDone = curve[0].done();
	// 当前总位移
	double moveS = preS - preBuf->procInfo.doneS;
	// 上一条轨迹插补完成
	if (preDone) {
		moveS += curS;
	}
	// 当前周期位移增量
	double detS = moveS - curBuf->interpInfo.curMoveS;

	// 当前插补比例
	double ratio = curS / (curBuf->procInfo.preBlendDist + curBuf->procInfo.mainDist + curBuf->procInfo.postBlendDist);

	// --- 计算当前位置
	pos.rbtPos = curBuf->pointInfo.begPos.rbtPos;
	// 当前曲线参数，离开平滑段后复位
	double curU = 0.0;
	// 前平滑段
	if (curBuf->procInfo.preSmooth > 0 && moveS < curBuf->procInfo.segmBegDist) {
		if (curBuf->interpInfo.partId != 1) {
			curBuf->interpInfo.partId = 1;
		}
		double curPos[3];
		curU = bezier_interp(5, curBuf->procInfo.preCtrlPnt, curBuf->interpInfo.curU, detS, 2);
		bezier_positioin(5, curBuf->procInfo.preCtrlPnt, curU, curPos);

		for (int i = 0; i < 3; ++i) {
			pos.rbtPos[i] = curPos[i];
		}
	}
	else {
		// 前平滑段结束后，补偿贝塞尔弧长累积误差，保证切换到直线段时位置连续
		// 但是这里计算的距离与规划时计算的距离不完全一致，若要保证终点完全一致最好重新规划
		if (curBuf->interpInfo.partId == 1 && curBuf->procInfo.preSmooth > 0) {
			double realDist = bezier_dist(5, curBuf->procInfo.preCtrlPnt, preBuf->procInfo.doneU, curBuf->interpInfo.curU, 1000);
			double error = curBuf->interpInfo.curMoveS - realDist;

			// 仅修正直线段入口，让直线段的 lambda 插值自然吸收误差
			// 直线段终点固定为 endPos，segmEndDist 不变则后平滑入口不受影响
			curBuf->procInfo.segmBegDist += error;
		}

		// 后平滑段
		if (curBuf->procInfo.postSmooth > 0 && moveS > curBuf->procInfo.segmEndDist) {
			// 后平滑段上的位移增量
			double dis = detS;

			if (curBuf->interpInfo.partId != 3) {
				curBuf->interpInfo.partId = 3;
				curBuf->interpInfo.curU = 0;
				//dis = moveS - curBuf->procInfo.segmBegDist - curBuf->procInfo.mainDist;
				// 减去直线段剩余长度
				dis -= curBuf->procInfo.segmEndDist - curBuf->interpInfo.curMoveS;
			}

			double curPos[3];
			curU = bezier_interp(5, curBuf->procInfo.postCtrlPnt, curBuf->interpInfo.curU, dis, 2);
			bezier_positioin(5, curBuf->procInfo.postCtrlPnt, curU, curPos);

			for (int i = 0; i < 3; ++i) {
				pos.rbtPos[i] = curPos[i];
			}

			// 后平滑输出第一个点后结束当前段插补，剩下的部分再下一条轨迹中插补
			curDone = true;
		}
		// 无平滑段
		else {
			// 无平滑段长度
			double dis = moveS - curBuf->procInfo.segmBegDist + curBuf->procInfo.preSmoothK * curBuf->procInfo.dist;
			double lambda = (curBuf->procInfo.dist < dim_EPS) ? 0.0 : dis / curBuf->procInfo.dist;

			if (curBuf->interpInfo.partId != 2) {
				curBuf->interpInfo.partId = 2;
			}

			calc_cartesian_breakpoint(curBuf, lambda, curBuf->motionCfg.moveType - 1, pos.rbtPos.data());
		}
	}

	// --- 附加轴插补
	pos.extPos[0] = curve[6].get_pos(curBuf->procInfo.curTime);
	if (curBuf->procInfo.preSmooth > 0) {
		pos.extPos[0] += curvePre[6].get_pos(curBuf->procInfo.curTime) - curve[6].get_pos(0);
	}

	// --- 摆焊叠加
	double swingAdd = 0.0;
	SwingConfig swing = rtMotionCfg.swingParam;
	if (swing.enable) {
		// 半个摆动周期的时间
		double singleSwingTime = 0.5 / swing.freq;
		if (swingProc.state == 0) {
			swingProc.state = 1;
			curve[2].set_condition(swingProc.pos, swing.rightWidth, 0, 0);
			curve[2].plan_by_duration(singleSwingTime, singleSwingTime / 2, singleSwingTime / 10);
			curve[2].plan();
			swingProc.time = cycleTime;
			swingProc.pos = swing.rightWidth;
		}
		else {
			swingAdd = curve[2].get_pos(swingProc.time);

			if (curve[2].done()) {
				swingProc.state = swingProc.state % 2 + 1;

				// 剩余时间不足一个完整周期
				double remainSwingTime = swingProc.duration - curBuf->procInfo.curTime;
				bool timeReset = remainSwingTime < singleSwingTime * 2;
				if (timeReset)
					singleSwingTime = remainSwingTime;

				if (swingProc.state == 2) {
					curve[2].set_condition(swingProc.pos, timeReset ? 0 : -swing.leftWidth, 0, 0);
					swingProc.pos = -swing.leftWidth;
				}
				else {
					curve[2].set_condition(swingProc.pos, timeReset ? 0 : swing.rightWidth, 0, 0);
					swingProc.pos = swing.rightWidth;
				}

				curve[2].plan_by_duration(singleSwingTime, singleSwingTime / 2, singleSwingTime / 10);
				swingProc.time = 0.0;
			}
			swingProc.time += cycleTime;
		}
		rtMotionCfg.swingParam = swing;
	}

	// 实时状态
	interpStatus.vel = 0.0;
	for (int i = 0; i < 3; ++i) {
		interpStatus.tan[i] = pos.rbtPos[i] - interpStatus.cPos[i];
		interpStatus.cPos[i] = pos.rbtPos[i];
		interpStatus.vel += interpStatus.tan[i] * interpStatus.tan[i];
	}
	interpStatus.vel = sqrt(interpStatus.vel);
	for (int i = 0; i < 3; ++i) {
		interpStatus.tan[i] /= interpStatus.vel;
	}
	// 叠加到插补坐标系的Y方向
	pos.rbtPos[0] += swingAdd * interpStatus.tan[1];
	pos.rbtPos[1] -= swingAdd * interpStatus.tan[0];

	// --- 插补状态更新
	// 插补进度
	curBuf->interpInfo.schedule = ratio;
	// 当前目标位置
	curBuf->interpInfo.dpos = pos;
	curBuf->interpInfo.curMoveS = moveS;
	curBuf->interpInfo.curU = curU;

	// 插补完成: 有平滑时，剩余规划时间小于一个插补周期，将剩余距离移到下条轨迹前平滑段内插补
	if (curDone) {
		curBuf->interpInfo.partId = -1;
		// 下一段平滑从当前后平滑继续
		nextBuf->interpInfo.curU = curBuf->interpInfo.curU;
		curBuf->procInfo.doneS = curS;
		curBuf->procInfo.doneU = curU;
	}

	// 插补时间累加
	curBuf->procInfo.curTime += cycleTime;

	return 0;
}

double InterpBuffer::cartesian_look_ahead() {
	// 1. 记录前瞻减速点: 从当前点可以正常运动加速到目标点
	// 2. 记录后溯减速点: 从当前点可以正常运动并停在结束点
	// 3. 合并减速点，得到最终减速点的速度
	// 4. 基于第一个减速点，计算到达当前段后平滑曲率点的终点速度
	// 5. 基于当前段/下一段设置速度和自适应速度约束Ve值

	// 轨迹前瞻，队首开始
	InterpSegment *preBuf = nullptr, *curBuf = nullptr, *nextBuf = nullptr;
	get_neighbor_buffer(bufBeg, preBuf, curBuf, nextBuf);

	// 最大前瞻段数, 实际前瞻段数: forwardNum = 2表示除了当前段，还有两段需要前瞻
	int maxForwardNum = 10, forwardNum = 0;
	// 当前段有后平滑，启用前瞻
	if (curBuf->procInfo.postSmooth > 0) {
		for (int i = 0; i < maxForwardNum; ++i) {
			forwardNum++;

			InterpSegment *tmpBuf = get_following_buffer(bufBeg, i+1);
			if (tmpBuf->procInfo.postSmooth < dim_EPS)
				break;
		}
	}
	// 无需前瞻, 结束点速度为0
	if (forwardNum == 0)
		return 0.0;

	DoubleSCurve curve;
	// 前瞻/回溯段起点速度
	double vsForward = preBuf->procInfo.constrainedVel;
	// 第i段终点限速，当前段起点和最后一段终点速度(0)均已知
	std::vector<double> vlim(forwardNum, 0.0);

	// --- 前瞻: 从当前点加速
	for (int i = 0; i < forwardNum; ++i) {
		InterpSegment *tmpBuf = get_following_buffer(bufBeg, i);
		double dist = (i == 0) ? preBuf->procInfo.remainS : 0.0;
		dist += tmpBuf->procInfo.preBlendDist + tmpBuf->procInfo.mainDist + tmpBuf->procInfo.postBlendDist;

		// 最大提速速度
		curve.set_constraint(tmpBuf->motionCfg.speed, 10, 10 * 10);
		curve.set_condition(0, dist, vsForward, 0);
		double maxSpeed = curve.get_max_speed(dist);

		// 前瞻约束速度
		vlim[i] = std::min(maxSpeed, tmpBuf->procInfo.constrainedVel);
		vsForward = vlim[i];
	}

	// --- 回溯: 从终点加速, i+1 段起点速度即为 i 段终点速度
	vsForward = 0.0;
	for (int i = forwardNum - 1; i >= 0; --i) {
		InterpSegment *tmpBuf = get_following_buffer(bufBeg, i + 1);
		double dist = tmpBuf->procInfo.preBlendDist + tmpBuf->procInfo.mainDist + tmpBuf->procInfo.postBlendDist;

		// 最大提速速度
		curve.set_constraint(tmpBuf->motionCfg.speed, 10, 10 * 10);
		curve.set_condition(0, dist, vsForward, 0);
		double maxSpeed = curve.get_max_speed(dist);

		// 回溯约束速度
		vlim[i] = std::min(maxSpeed, vlim[i]);
		vsForward = vlim[i];
	}

	return vlim[0];
}

int InterpBuffer::set_jog_constraint(int idx, double q0, double vmax, double amax, double jmax) {
	curve[idx].set_condition(q0, q0, 0, 0);
	curve[idx].set_constraint(vmax, amax, jmax);
	return 0;
}

int InterpBuffer::switch_jog_state(int idx, int state) {
	curve[idx].set_onlineState(state);
	return 0;
}

int InterpBuffer::jog_move(int idx) {
	int state = 0;
	// 细化插补，减少突变，按500us插补，按实际周期输出
	int num = cycleTime / 5e-4;
	num = 1;
	for (int i = 0; i < num; ++i) {
		state = curve[idx].online_interp(cycleTime / num);
	}

	return state;
}

int InterpBuffer::get_online_interp_result(int idx, double ans[4]) {
	curve[idx].get_cur_state(ans);
	return 0;
}

double InterpBuffer::plan_decccel_online_interp(int idx) {
	// 获取当前状态
	double xt[4] = { 0.0 };
	int onlineState = curve[idx].get_cur_state(xt);

	double endmove = xt[0];
	if (std::abs(onlineState) == 1) {
		double Tdi[3];
		endmove = curve[idx].plan_decccel_online_interp(Tdi);
	}
	// 减速阶段，返回终点值
	else if (std::abs(onlineState) == 2) {
		endmove = curve[idx].get_q1();
	}

	return endmove;
}
