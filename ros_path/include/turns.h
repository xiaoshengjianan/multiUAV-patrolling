#pragma once
#include<vector>
#include<numeric>
#include<math.h>
class turns {
public:
	double avg = 0;
	double standard_error = 0;
	std::vector<std::vector<std::vector<int>>> path;
	std::vector<int> turn;
	void count_turns();
	turns(std::vector<std::vector<std::vector<int>>> apath);
	void cal_avgstd();
};