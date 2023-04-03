#include "turns.h"
turns::turns(std::vector<std::vector<std::vector<int>>> apath) {
	path = apath;
}
void turns::count_turns() {
    int num_turns = -1;
    int last_move = -1;
    int current_move = 0;
    for (int i = 0; i < path.size(); ++i) {//取一架无人机的路径
        for (int j = 0; j < path[i].size(); ++j) {//取一架无人机的多次移动计算转弯
            if (path[i][j][0] == path[i][j][2]) {//水平方向移动
                current_move = 0;
            }
            else if (path[i][j][1] == path[i][j][3]) {
                current_move = 1;//垂直方向移动
            }
            if (last_move != current_move) {
                num_turns += 1;
                last_move = current_move;
            }
        }
        turn.push_back(num_turns);
    }
}
void turns::cal_avgstd() {
    avg = accumulate(turn.begin(), turn.end(), 0)/turn.size();
    double tmp = 0.0;
    for (int i = 0; i < turn.size(); ++i) {
        tmp += (turn[i] - avg)* (turn[i] - avg);
    }
    standard_error = tmp / sqrt(turn.size()-1);
}