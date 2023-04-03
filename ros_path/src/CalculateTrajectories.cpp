#include"CalculateTrajectories.h"
CalculateTrajectories::CalculateTrajectories(int r, int c, std::vector<Edge>MST) {
	MAX_NODES = 4 * r * c;//4*r*c�Ƿָ��С����,�����r��submap��row��1/2
	rows = r;
	cols = c;
	MSTvector = MST;
	MSTedges = MSTvector.size();
	nodes.resize(MAX_NODES);
}
void CalculateTrajectories::InitializeGraph() {
	for (int i = 1; i <= 2 * rows; ++i) {
		for (int j = 1; j <= 2 * cols; ++j) {
			if (i > 1)Add2AllEdges(allEdges,j-1 + (i - 1) * 2 * cols, j-1 + (i - 2) * 2 * cols, 1);
			if (i < 2 * rows)Add2AllEdges(allEdges,j-1 + (i - 1) * 2 * cols, j-1 + (i) * 2 * cols, 1);
			if (j > 1)Add2AllEdges(allEdges,j-1 + (i - 1) * 2 * cols, (i - 1) * 2 * cols + j-2, 1);
			if (j < 2 * cols)Add2AllEdges(allEdges,j-1 + (i - 1) * 2 * cols, (i - 1) * 2 * cols + j, 1);
		}
	}
}
void CalculateTrajectories::Add2AllEdges(std::set<Edge>& allEdges,int from, int to, int cost) {
	Edge edge(from, to, cost);
	allEdges.insert(edge);
	nodes[from].insert(to);
	nodes[to].insert(from);
}
void CalculateTrajectories::RemoveTheAppropriateEdges() {
	for (int i = 0; i < MSTedges; ++i) {
		Edge e = MSTvector[i];
		int maxN = e.src > e.dst ? e.src : e.dst;
		int minN = e.src < e.dst ? e.src : e.dst;
		if (std::abs(e.src - e.dst) == 1)
		{
			int alpha = (4 * minN + 3) - 2 * (maxN % cols);
			Edge eToRemove(alpha, alpha + 2 * cols, 1);//ȥ����Ӧ����ͨ�Ĳ���,���㵽С���Ӷ�Ӧ������ֵ
			Edge eToRemoveMirr(alpha + 2 * cols, alpha, 1);
			Edge eToRemove2(alpha + 1, alpha + 1 + 2 * cols, 1);
			Edge eToRemove2Mirr(alpha + 1 + 2 * cols, alpha + 1, 1);
			if (SearchEdge(eToRemove)) {
				SafeRemoveEdge(eToRemove);
			}
			if (SearchEdge(eToRemoveMirr)) {
				SafeRemoveEdge(eToRemoveMirr);
			}
			if (SearchEdge(eToRemove2)) {
				SafeRemoveEdge(eToRemove2);
			}
			if (SearchEdge(eToRemove2Mirr)) {
				SafeRemoveEdge(eToRemove2Mirr);
			}
		}
		else {
			int alpha = (4 * minN + 2 * cols) - 2 * (maxN % cols);
			Edge eToRemove(alpha, alpha + 1, 1);//ȥ����Ӧ����ͨ�Ĳ���
			Edge eToRemoveMirr(alpha + 1, alpha, 1);
			Edge eToRemove2(alpha + 2 * cols, alpha + 1 + 2 * cols, 1);
			Edge eToRemove2Mirr(alpha + 1 + 2 * cols, alpha + 2 * cols, 1);
			if (SearchEdge(eToRemove)) {
				SafeRemoveEdge(eToRemove);
			}
			if (SearchEdge(eToRemoveMirr)) {
				SafeRemoveEdge(eToRemoveMirr);
			}
			if (SearchEdge(eToRemove2)) {
				SafeRemoveEdge(eToRemove2);
			}
			if (SearchEdge(eToRemove2Mirr)) {
				SafeRemoveEdge(eToRemove2Mirr);
			}
		}
	}
}
void CalculateTrajectories::SafeRemoveEdge(Edge ed) {
	if (SearchEdge(ed)) {
		std::set<Edge>::iterator t;
		for (t = allEdges.begin(); t != allEdges.end(); ++t) {
			if (t->dst == ed.dst && t->src == ed.src && t->weight == ed.weight) {
				allEdges.erase(t++);
				break;
			}
		}
	}
	else {
		std::cout << "treeset should contain this element" << std::endl;
	}
	if (SearchNode(nodes[ed.src], ed.dst)) {
		nodes[ed.src].erase(ed.dst);
	}
	if (SearchNode(nodes[ed.dst], ed.src)) {
		nodes[ed.dst].erase(ed.src);
	}
}
bool CalculateTrajectories::SearchEdge(Edge ed) {
	std::set<Edge>::iterator t;
	for (t = allEdges.begin(); t != allEdges.end(); ++t) {
		if (t->dst == ed.dst && t->src == ed.src && t->weight == ed.weight) {
			return true;
		}
	}
	return false;
}
bool  CalculateTrajectories::SearchNode(std::vector<std::set<int>>&nodes, int node) {
	for (int i = 0; i < nodes.size(); ++i) {
		std::set<int>::iterator t;
		t = std::find(nodes[i].begin(), nodes[i].end(), node);
		if (t != nodes[i].end())return true;
	}
	return false;//û�ҵ�
}
bool  CalculateTrajectories::SearchNode(std::set<int>vec, int node) {

	std::set<int>::iterator t;
	t = std::find(vec.begin(), vec.end(), node);
	if (t != vec.end())return true;
	return false;//
}
bool  CalculateTrajectories::SearchNode(std::vector<int>vec, int node) {

	std::vector<int>::iterator t;
	t = std::find(vec.begin(), vec.end(), node);
	if (t != vec.end())return true;
	return false;//
}
bool  CalculateTrajectories::CalculatePathsSequence(int startingnode) {
	int currentnode = startingnode;
	std::set<int>removednodes;
	std::vector<int>movement;
	movement.push_back(2 * cols);
	movement.push_back(-1);
	movement.push_back(-2 * cols);
	movement.push_back(1);
	bool found = false;
	int prevnode = 0;
	for (int i = 0; i < 4; ++i) {
		int tmp_node = currentnode + movement[i];
		if (SearchNode(nodes, tmp_node)) {
			prevnode = currentnode + movement[i];
			found = true;
			break;
		}
	}
	if (not found)return 0;
	while (1) {
		if (currentnode != startingnode)removednodes.insert(currentnode);
		int tmpn = prevnode - currentnode;
		std::vector<int>::iterator result = std::find(movement.begin(), movement.end(), tmpn);//�ҵ�һ������
		int offset = std::distance(movement.begin(), result);
		prevnode = currentnode;
		found = false;
		for (int idx = 0; idx < 4; ++idx) {//�ҵڶ�������
			if (SearchNode(nodes[prevnode], prevnode + movement[(idx + offset) % 4]) && !SearchNode(removednodes, prevnode + movement[(idx + offset) % 4])) {
				currentnode = prevnode + movement[(idx + offset) % 4];
				found = true;
				break;
			}
		}
		if (not found)return 0;
		if (SearchNode(nodes[currentnode], prevnode))nodes[currentnode].erase(prevnode);
		if (SearchNode(nodes[prevnode], currentnode))nodes[prevnode].erase(currentnode);
		int i = int(currentnode / (2 * cols));
		int j = currentnode % (2 * cols);
		int previ = int(prevnode / (2 * cols));
		int prevj = prevnode % (2 * cols);
		std::vector<int> v;//�ֲ�������һ��ѭ��֮�����ʧ
		v.push_back(previ);
		v.push_back(prevj);
		v.push_back(i);
		v.push_back(j);
		PathSequence.push_back(v);
	}
};
