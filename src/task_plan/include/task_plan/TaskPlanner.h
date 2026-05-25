#pragma once

#include "task_plan/TaskTrajectory.h"

namespace task_plan {

/*
 * @brief 检测路径可行性, 过滤不合理路径
 * 
 * @param trajectories 输入路径
 * @return true 所有路径都可行
 * @return false 有路径不可行
 */
bool check_path_feasibility(const std::vector<TaskTrajectory>& trajectories);

// 路径分组和排序
int group_and_sort_paths(std::vector<std::vector<TaskTrajectory>>& trajectories);

} // namespace task_plan
