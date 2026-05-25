
#include "task_plan/collision.h"

// 引入 FCL 的核心头文件
//#include <fcl/narrowphase/collision.h>

// 引入 Coal 的核心头文件
// #include "coal/math/transform.h"

namespace task_plan {

bool check_trajectory_feasibility(const TaskTrajectory& trajectory) {
	return true;
}

std::vector<TaskTrajectory>
	optimize_trajectory(const TaskTrajectory& trajectory) {
	return { trajectory };
}

} // namespace task_plan
