#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state)
{
	// cast the abstract state type to the type we expect
	const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

	// extract the first component of the state and cast it to what we expect
	const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

	// extract the second component of the state and cast it to what we expect
	const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

	// check validity of state defined by pos & rot

	// return a value that is always true but uses the two variables we define, so we avoid compiler warnings
	return (const void*)rot != (const void*)pos;
}

void plan()
{
	// construct the state space we are planning in
	auto space(std::make_shared<ob::SE3StateSpace>());

	// set the bounds for the R^3 part of SE(3)
	ob::RealVectorBounds bounds(3);
	bounds.setLow(-1);
	bounds.setHigh(1);

	space->setBounds(bounds);

	// construct an instance of  space information from this state space
	auto si(std::make_shared<ob::SpaceInformation>(space));

	// set state validity checking for this space
	si->setStateValidityChecker(isStateValid);

	// create a random start state
	ob::ScopedState<> start(space);
	start.random();

	// create a random goal state
	ob::ScopedState<> goal(space);
	goal.random();

	// create a problem instance
	auto pdef(std::make_shared<ob::ProblemDefinition>(si));

	// set the start and goal states
	pdef->setStartAndGoalStates(start, goal);

	// create a planner for the defined space
	auto planner(std::make_shared<og::RRTConnect>(si));

	// set the problem we are trying to solve for the planner
	planner->setProblemDefinition(pdef);

	// perform setup steps for the planner
	planner->setup();


	// print the settings for this space
	si->printSettings(std::cout);

	// print the problem settings
	pdef->print(std::cout);

	// attempt to solve the problem within one second of planning time
	ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

	if (solved)
	{
		// get the goal representation from the problem definition (not the same as the goal state)
		// and inquire about the found path
		ob::PathPtr path = pdef->getSolutionPath();
		std::cout << "Found solution:" << std::endl;

		// print the path to screen
		path->print(std::cout);
	}
	else
		std::cout << "No solution found" << std::endl;
}

void planWithSimpleSetup()
{
	// construct the state space we are planning in
	auto space(std::make_shared<ob::SE3StateSpace>());

	// set the bounds for the R^3 part of SE(3)
	ob::RealVectorBounds bounds(3);
	bounds.setLow(-1);
	bounds.setHigh(1);

	space->setBounds(bounds);

	// define a simple setup class
	og::SimpleSetup ss(space);

	// set state validity checking for this space
	ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });

	// create a random start state
	ob::ScopedState<> start(space);
	start.random();

	// create a random goal state
	ob::ScopedState<> goal(space);
	goal.random();

	// set the start and goal states
	ss.setStartAndGoalStates(start, goal);

	// this call is optional, but we put it in to get more output information
	ss.setup();
	ss.print();

	// attempt to solve the problem within one second of planning time
	ob::PlannerStatus solved = ss.solve(1.0);

	if (solved)
	{
		std::cout << "Found solution:" << std::endl;
		// print the path to screen
		ss.simplifySolution();
		ss.getSolutionPath().print(std::cout);
	}
	else
		std::cout << "No solution found" << std::endl;
}


bool isStateValidTest(const ob::State *state)
{
	// cast the abstract state type to the type we expect
	const ob::RealVectorStateSpace::StateType& pos = *state->as<ob::RealVectorStateSpace::StateType>();

	// extract the first component of the state and cast it to what we expect
	//const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
	double x = pos[0], y = pos[1];

	// extract the second component of the state and cast it to what we expect
	//const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

	// check validity of state defined by pos & rot
	bool valid = y < x * x;
	if (y < x * x - 1.0) {
		valid = false;
	}

	// return a value that is always true but uses the two variables we define, so we avoid compiler warnings
	return valid;
}

void test_plan() {
	// construct the state space we are planning in
	auto space(std::make_shared<ob::RealVectorStateSpace>(2));

	// set the bounds for the R^2 part of SE(2)
	ob::RealVectorBounds bounds(2);
	bounds.setLow(0, -1.0);
	bounds.setHigh(0, 1.0);
	bounds.setLow(1, -0.1);
	bounds.setHigh(1, 1.0);
	space->setBounds(bounds);

	// construct an instance of  space information from this state space
	auto si(std::make_shared<ob::SpaceInformation>(space));

	// set state validity checking for this space
	si->setStateValidityChecker(isStateValidTest);

	// create a random start state
	ob::ScopedState<> start(space);
	start[0] = -1.0;
	start[1] = 0.5;

	ob::ScopedState<> goal(space);
	goal[0] = 1.0;
	goal[1] = 0.8;

	// create a problem instance
	auto pdef(std::make_shared<ob::ProblemDefinition>(si));

	// set the start and goal states
	pdef->setStartAndGoalStates(start, goal);

	// create a planner for the defined space
	auto planner(std::make_shared<og::RRTConnect>(si));

	// set the problem we are trying to solve for the planner
	planner->setProblemDefinition(pdef);

	// perform setup steps for the planner
	planner->setup();


	// print the settings for this space
	si->printSettings(std::cout);

	// print the problem settings
	pdef->print(std::cout);

	// attempt to solve the problem within one second of planning time
	ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

	if (!solved) {
		std::cout << "No solution found" << std::endl;
		return;
	}

	// get the goal representation from the problem definition (not the same as the goal state)
	// and inquire about the found path
	ob::PathPtr path = pdef->getSolutionPath();
	std::cout << "Found solution:" << std::endl;

	// print the path to screen
	path->print(std::cout);

	std::ofstream output("output.txt");
	std::ofstream fineOutPut("fine_output.txt");
	if (!output) {
		return;
	}
	// 向下转型为几何路径指针
	// 注意：请确保你的规划器求解的是几何路径（如 RRT、PRM 等）
	ompl::geometric::PathGeometric* geom_path = path->as<ompl::geometric::PathGeometric>();
	const std::vector<ompl::base::State*>& states = geom_path->getStates();
	for (int i = 0; i < states.size(); ++i) {
		ompl::base::State* state = states[i];

		// 在这里处理每个 state，例如打印或转换为具体类型
		// 需要根据你的状态空间类型来提取坐标值
		// 例如，对于 RealVectorStateSpace:
		const auto* rv_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
		std::cout << (*rv_state)[0] << ", " << (*rv_state)[1] << std::endl;
		output << (*rv_state)[0] << ", " << (*rv_state)[1] << std::endl;
	}


	// 初始规划路径后处理，全局剪枝
	int pre = 0, end = pre + 1;
	std::vector<int> fineIdx = { pre };
	for (int i = 1; i < states.size(); ++i) {
		if (!si->checkMotion(states[pre], states[i])) {
			std::cout << i << ", 路径上有碰撞！" << std::endl;
			fineIdx.push_back(i - 1);
			pre = i - 1;
		}
	}
	fineIdx.push_back(states.size() - 1);

	for (int i = 0; i < fineIdx.size(); ++i) {
		ompl::base::State* state = states[fineIdx[i]];

		const auto* rv_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
		fineOutPut << (*rv_state)[0] << ", " << (*rv_state)[1] << std::endl;
	}
}

int main(int /*argc*/, char ** /*argv*/)
{
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

	test_plan();

	std::cout << std::endl << std::endl;

	//planWithSimpleSetup();

	return 0;
}