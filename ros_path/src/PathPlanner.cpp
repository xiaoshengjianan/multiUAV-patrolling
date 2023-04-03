#include "PathPlanner.h"
#include"Kruskal.h"
#include"CalculateTrajectories.h"
//#define INT_MAX 0x7fffffff
PathPlanner::PathPlanner(Drone_Vec drones) {
	std::vector<vector3i>AllRealPaths_dict;
	for (int i = 0; i < 4; ++i) {//mode有4种
		CalculateMSTs(drones, i);
		std::vector<std::vector<std::vector<int>>>().swap(allpath);
		for (int j = 0; j < drones.drone_vec.size(); ++j) {
			CalculateTrajectories C(drones.drone_vec[j].mymap.subrow / 2, drones.drone_vec[j].mymap.subcol / 2, MSTs[j]);
			C.InitializeGraph();
			C.RemoveTheAppropriateEdges();
			C.CalculatePathsSequence(0);//默认从0出发
			allpath.push_back(C.PathSequence);//pathsequence中储存一架无人机的路径段，allpath中储存所有1无人机的路径
		}
		//int*** typelines;
		//typelines = new  int** [2];
		//for (int i = 0; i < 2; i++) {
		//	typelines[i] = new int* [drones.drone_vec[i].mymap.subrow];
		//	for (int j = 0; j < drones.drone_vec[i].mymap.subrow; j++) {
		//		typelines[i][j] = new int[drones.drone_vec[i].mymap.subcol];
		//	}
		//}
		//for (int i = 0; i < 2; i++) {
		//	for (int j = 0; j < drones.drone_vec[i].mymap.subrow; j++) {
		//		for (int k = 0; k < drones.drone_vec[i].mymap.subcol; k++) {
		//			typelines[i][j][k] = 0;//初始化
		//		}
		//	}
		//}
		//int indxadd1=0;
		//int indxadd2=0;
		//for (int r = 0; r < drones.drone_vec.size(); ++r) {
		//	bool flag = false;
		//	for (int i = 0; i < allpath[r].size(); ++i) {
		//		if (flag) {
		//			if (typelines[allpath[r][i][0]][allpath[r][i][1]][0] == 0) {//prev点的x和y
		//				int idxadd1 = 0;
		//			}
		//			else int idexadd1 = 1;
		//			if (typelines[allpath[r][i][2]][allpath[r][i][3]][0] == 0 && flag) {//current点的x和y
		//				int idxadd2 = 0;
		//			}
		//			else int idexadd2 = 1;
		//		}
		//		else {
		//			if (typelines[allpath[r][i][0]][allpath[r][i][1]][0] != 0) {
		//				int idxadd1 = 0;
		//			}
		//			else int idexadd1 = 1;
		//			if (!(typelines[allpath[r][i][2]][allpath[r][i][3]][0] == 0 && flag)) {
		//				int idxadd2 = 0;
		//			}
		//			else int idexadd2 = 1;
		//		}
		//		flag = true;
		//		if (allpath[r][i][0] == allpath[r][i][2]) {//prenode的x与currentnode的x相等
		//			if (allpath[r][i][1] > allpath[r][i][3]) {//prenode的y大于currentnode的y
		//				typelines[allpath[r][i][0]][allpath[r][i][1]][indxadd1] = 2;
		//				typelines[allpath[r][i][2]][allpath[r][i][3]][indxadd2] = 3;
		//			}
		//			else {
		//				typelines[allpath[r][i][0]][allpath[r][i][1]][indxadd1] = 3;
		//				typelines[allpath[r][i][2]][allpath[r][i][3]][indxadd2] = 2;
		//			}
		//		}
		//		else {
		//			if (allpath[r][i][0] > allpath[r][i][2]) {
		//				typelines[allpath[r][i][0]][allpath[r][i][1]][indxadd1] = 1;
		//				typelines[allpath[r][i][2]][allpath[r][i][3]][indxadd2] = 4;
		//			}
		//			else {
		//				typelines[allpath[r][i][0]][allpath[r][i][1]][indxadd1] = 4;
		//				typelines[allpath[r][i][2]][allpath[r][i][3]][indxadd2] = 1;
		//			}
		//		}
		//	}
		//}
		turns drone_turns(allpath);
		drone_turns.count_turns();
		drone_turns.cal_avgstd();
		mode_to_drone_turns.push_back(drone_turns);
		AllRealPaths_dict.push_back(allpath);
	}
	int tmp_sum = 0;
	int index=0;
	std::vector<double> averge_turns;
	for (int i = 0; i < mode_to_drone_turns.size(); ++i) {
		averge_turns.push_back(mode_to_drone_turns[i].avg);
	}
	double minturns=averge_turns[0];
	for (int i = 0; i < averge_turns.size(); ++i) {
		if (averge_turns[i] < minturns) {
			minturns = averge_turns[i];//寻找4种模式中最少转弯的一种
			index = i;
		}
	}
	minturns = INT_MAX;
	std::vector<std::vector<int>>tmp_path;
	std::vector<int> combined_modes_turns;
	std::vector<std::vector<std::vector<int>>>combined_modes_paths;
	for (int r = 0; r < drones.drone_vec.size(); ++r) {
		for (int i = 0; i < 4; ++i) {
			if (mode_to_drone_turns[i].turn[r] < minturns) {
				minturns = mode_to_drone_turns[i].turn[r];
				tmp_path = mode_to_drone_turns[i].path[r];
			}
		}
		combined_modes_paths.push_back(tmp_path);
		combined_modes_turns.push_back(minturns);
	}
	turns best_case(combined_modes_paths);
	best_case.turn = combined_modes_turns;
	best_case.cal_avgstd();
	std::vector<int>best_case_num_paths;
	for (int i = 0; i < drones.drone_vec.size(); ++i) {
		best_case_num_paths.push_back(best_case.path[i].size());//记录每架飞机的path长度
	}
	std::cout << "maximum number of cells in robots paths:" <<*std::max_element(best_case_num_paths.begin(),best_case_num_paths.end()) << std::endl;
	std::cout << "minimum number of cells in robots paths:" << *std::min_element(best_case_num_paths.begin(), best_case_num_paths.end()) << std::endl;
}
void PathPlanner::CalculateMSTs(Drone_Vec drones, int mode) {
	std::vector<std::vector<Edge>>().swap(MSTs);
	for (int i = 0; i < drones.drone_vec.size(); ++i) {
		int row = drones.drone_vec[i].GetMyMap().subrow/2;
		int col = drones.drone_vec[i].GetMyMap().subcol/2;
		Kruskal K(row,col);
		K.InitializeGraph( mode);//到这一步都对
		K.PerformKruskal();
		MSTs.push_back(K.mst);
	}
}

