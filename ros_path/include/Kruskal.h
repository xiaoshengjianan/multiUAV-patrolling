#pragma once
#include<vector>
#include"Edge.h"
#include<set>
class Kruskal {
public:
	int rows;
	int cols;
	int MAX_NODES;
	std::vector<Edge>mst;
	std::vector<Edge>allEdges;
	std::vector<std::set<int>>nodes1;
	Kruskal(int r, int c);
	void InitializeGraph(int mode);
	void Add2AllEdges(int from, int to, int cost);
	void PerformKruskal();
};
