#pragma once
#include<set>
#include<list>
#include"firststage.h"
#include"Edge.h"
#include <unsupported/Eigen/CXX11/Tensor>
#include"turns.h"
#include<algorithm>
typedef std::vector<std::vector<std::vector<int>>> vector3i;
class Edge;
class PathPlanner {//计算多个无人机的路径
public:
	int ny;
	int nx;
	std::vector<std::vector<Edge>> MSTs;
	PathPlanner(Drone_Vec drones);
	void CalculateMSTs(Drone_Vec drones, int mode);
	vector3i allpath;
	std::vector<turns>mode_to_drone_turns;
	int min_mode;
};



