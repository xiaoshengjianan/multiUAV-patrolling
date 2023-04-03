#include"Kruskal.h"
Kruskal::Kruskal(int r, int c) {
	rows = r;
	cols = c;
	MAX_NODES = rows * cols;
	nodes1.resize(MAX_NODES);
}
void Kruskal::InitializeGraph(int mode) {//各边的权重初始化,用于去除小格子不连通的部分
	int cost1 = 1;
	int cost2 = 1;
	int maxnode = rows * cols;
	nodes1.resize(maxnode);
	for (int i = 1; i <= rows; ++i) {
		for (int j = 1; j <= cols; ++j) {
			if (mode == 0)cost2 = rows - i + 1;
			else if (mode == 1)cost2 = i;
			else if (mode == 2)cost1 = cols - j + 1;
			else if (mode == 3)cost1 = j;
			if (i > 1)Add2AllEdges(j-1 + (i - 1) * cols, j-1 + (i - 2) * cols, cost1);//下
			if (i < rows)Add2AllEdges(j-1 + (i - 1) * cols, j-1 + (i)*cols, cost1);//上
			if (j > 1)Add2AllEdges(j-1 + (i - 1) * cols, (i - 1) * cols + j - 2, cost2);//左
			if (j < cols)Add2AllEdges(j-1 + (i - 1) * cols, (i - 1) * cols + j, cost2);//右
		}
	}
}
void Kruskal::Add2AllEdges(int from, int to, int cost) {
	Edge edge(from, to, cost);
	allEdges.insert(allEdges.begin(),edge);
	if (nodes1[from].empty())nodes1[from].insert(from);
	if (nodes1[to].empty())nodes1[to].insert(to);
}
void Kruskal::PerformKruskal() {
	Graph g(nodes1, allEdges);
	g.KruskalMST();
	mst = g.mst;
}
