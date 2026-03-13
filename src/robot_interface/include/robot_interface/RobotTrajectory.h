#pragma once

#include <eigen3/Eigen/Dense>
#include <list>
#include <vector>
#include <map>


//namespace RobotTrajectory {

typedef float DT_scale;
const DT_scale DT_PI = 3.14159265358979323846;


// 计算等效的 zyx 欧拉角
Eigen::Matrix<DT_scale, 3, 1> get_equivalent_zyx_euler(const Eigen::Matrix<DT_scale, 3, 1>& endEuler);

// 计算两组 zyx 欧拉角之间的相对距离
Eigen::Matrix<DT_scale, 3, 1> get_zyx_euler_distance(Eigen::Matrix<DT_scale, 3, 1>& begEuler, Eigen::Matrix<DT_scale, 3, 1>& endEuler, bool chooseMimumDist = true);

// 计算两组 zyx 欧拉角之间的相对距离，经过中间点
Eigen::Matrix<DT_scale, 3, 1> get_zyx_euler_distance(Eigen::Matrix<DT_scale, 3, 1>& begEuler, Eigen::Matrix<DT_scale, 3, 1>& midEuler, Eigen::Matrix<DT_scale, 3, 1>& endEuler);

/**
* @brief  计算轨迹信息
* @param  begPnt    起点位置
* @param  midPnt    中间点位置
* @param  endPnt    终点位置
* @param  mode      轨迹类型, 0 - 直线, 1 - 圆弧
* @return 直线: 起点, 长度, 方向
*         圆弧: 圆心, 半径, 法向
*/
std::vector<DT_scale> calc_traj_info(const std::vector<DT_scale>& begPnt, const std::vector<DT_scale>& midPnt, const std::vector<DT_scale>& endPnt, int mode);


// 轨迹类型
enum class TrajType {
	// 未定义
	None,

	// 绝对运动
	//JointABS, LineABS, ArcABS,
	// 相对运动
	Joint, Line, Arc,

	// 机器人坐标系位置
	Line_R, Arc_R
};


struct TrajectoryPoint {
	// 轨迹类型
	TrajType trajType = TrajType::None;

	// 轨迹点数据
	std::vector<DT_scale> mainPoint;
	// 补充数据，如圆弧运动的中间点等
	std::vector<DT_scale> auxPoint;

	TrajectoryPoint();
	TrajectoryPoint(int num);

	inline void set_trajType(TrajType type) {
		trajType = type;
	}
	inline TrajType get_trajType() const {
		return trajType;
	}
	
	inline void set_mainPoint(std::vector<DT_scale>& point) {
		mainPoint = point;
	}
	inline std::vector<DT_scale> get_mainPoint() const {
		return mainPoint;
	}
	
	inline void set_auxPoint(std::vector<DT_scale>& point) {
		auxPoint = point;
	}
	inline std::vector<DT_scale> set_auxPoint() const {
		return auxPoint;
	}

	// 与给定点相近
	//bool isClose(const TrajectoryPoint& other) const;

	/**
	* @brief  查询是否是关节运动
	* @return 查询结果
	*/
	int isJoint() const;
	int isLine() const;
	int isArc() const;
	int isCartesian() const;
	int isBaseMotion() const;
};


struct TrajectoryConfig {
	// 速度
	DT_scale speed = 10;
	// 平滑度
	DT_scale smooth = 0;
	// 机器人坐标系下运动
	bool moveInBase = false;
	// 轴号掩码
	std::vector<int> axisMask;
	// 修改编号
	int rewriteId = -1;
	// 允许唤醒
	int notifyEnable = 0;
	//! 保存编号
	int saveSeq = -1;
	// 等待起弧: 当前轨迹下发后，进入等待，起弧成功后才下发后续的轨迹
	int waitArcOn = 0;

	// 协同段总距离
	DT_scale syncDist = 0;

	// 自定义参数
	std::map<int, std::vector<DT_scale>> appendix;

public:
	TrajectoryConfig() {}

	int clear();

	inline int reset(DT_scale speed_, DT_scale smooth_) {
		speed = speed_;
		smooth = smooth_;
		return 0;
	}

	inline int set_speed(DT_scale speed_) {
		speed = speed_;
		return 0;
	}
	inline DT_scale get_speed()  const {
		return speed;
	}

	inline int set_smooth(DT_scale smooth_) {
		smooth = smooth_;
		return 0;
	}
	inline int get_smooth()  const {
		return smooth;
	}

	inline int set_moveInBase(bool enable) {
		moveInBase = enable;
		return 0;
	}
	inline bool get_moveInBase() const {
		return moveInBase;
	}

	// e.g. { 0, 1, 6，8 }
	inline void set_axisMask(const std::vector<int>& mask) {
		axisMask = mask;
	}
	inline std::vector<int> get_axisMask() const {
		return axisMask;
	}

	inline void set_syncDist(DT_scale dist) {
		syncDist = dist;
	}
	inline DT_scale get_syncDist() {
		return syncDist;
	}

	int add_appendix(const std::pair<int, std::vector<DT_scale>>& appendix_) {
		appendix[appendix_.first] = appendix_.second;
		return 0;
	}
	inline std::map<int, std::vector<DT_scale>> get_appendix() {
		return appendix;
	}
};


class SingleTrajectory : public TrajectoryPoint, public TrajectoryConfig {
public:
	//! 轨迹编号
	int lineNum = 0;
	//! 轨迹类型: 指令轨迹(0), 弧坑回填轨迹(1)
	int taskId = 0;

	SingleTrajectory();
	SingleTrajectory(const TrajectoryPoint& pnt, const TrajectoryConfig cfg);


	void set_point(const TrajectoryPoint& newPoint);
	TrajectoryPoint get_point() const;

	void set_config(const TrajectoryConfig& cfg);
	TrajectoryConfig get_config();

};


class DiscreteTrajectory {
	//! 轨迹序号
	//int trajNum = -1;
	// 当前段轨迹数据
	DT_scale dist;
	std::vector<DT_scale> dir;
	std::vector<DT_scale> knot;
	// 上一条轨迹
	SingleTrajectory preTraj;

public:
	// 轨迹链表
	std::list<SingleTrajectory> trajList;

	DiscreteTrajectory();

	// 添加轨迹
	int moveJABS(const std::vector<DT_scale>& end, const TrajectoryConfig& config);
	int moveLABS(const std::vector<DT_scale>& end, const TrajectoryConfig& config);
	int moveCABS(const std::vector<DT_scale>& mid, const std::vector<DT_scale>& end, const TrajectoryConfig& config);

	// 添加连续轨迹
	int push_new_trajectory(const DiscreteTrajectory& newTraj);

	// 施加转换
	int apply_rotate(const Eigen::Matrix<DT_scale, 3, 3>& rotMat);


	// 当前轨迹长度
	inline int size() {
		return trajList.size();
	}
	// 轨迹指令压栈完成, empty
	bool trajectory_loaded();
	// 迭代器移动到下一条轨迹
	int next();
	void clear();
	// 当前轨迹是最后一条
	bool atLast() const;
	// 获取当前轨迹
	SingleTrajectory get_curTraj() const;
	// 获取上一条轨迹
	SingleTrajectory get_preTraj() const;
	// 获取下一条轨迹
	SingleTrajectory get_aftTraj() const;
	// 设置上一条轨迹
	int set_preTraj(const SingleTrajectory& traj);
	int set_preTraj(const TrajectoryPoint& point);
	// 修改当前轨迹
	int set_curTraj(const SingleTrajectory& traj);
	// 设置当条轨迹的序号
	inline int set_current_line_num(int num) {
		trajList.front().lineNum = num;
		return 0;
	}
	// 设定上条轨迹的序号
	inline int set_previous_line_num(int num) {
		preTraj.lineNum = num;
		return 0;
	}


	// 计算轨迹信息
	int calc_traj_info();
	// 获取相对距离
	std::vector<DT_scale> get_relative_distance();
	inline DT_scale get_dist() {
		return dist;
	}
	inline std::vector<DT_scale> get_dir() {
		return dir;
	}
	inline std::vector<DT_scale> get_knot() {
		return knot;
	}
};


/**
* @brief 分段轨迹
		------------------------------------------------
		|             ↑              ↑               |
		preTraj(0)     begRatio        endRatio        curTraj(1 or dist)
					  └--------------┘ ans
* @param  preTraj     前一条轨迹/起点
* @param  curTraj     目标轨迹/终点
* @param  begRatio    分段起点
* @param  endRatio    分段终点
* @param  mode        模式: 0-比例, 1-距离
* @return
*/
TrajectoryPoint partition_trajectory(const TrajectoryPoint& preTraj, const TrajectoryPoint& curTraj, DT_scale begRatio, DT_scale endRatio, int mode);

std::vector<DT_scale> get_relative_distance(const std::vector<DT_scale>& beg, const std::vector<DT_scale>& mid, const std::vector<DT_scale>& end);

bool make_circle(const std::vector<Eigen::Matrix<DT_scale, 3, 1>>& pts, Eigen::Matrix<DT_scale, 3, 1>& center);

//} // namespace RobotTrajectory