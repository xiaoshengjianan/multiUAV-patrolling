#include"Edge.h"
#include<algorithm>
Graph::Graph(std::vector<std::set<int>>n, std::vector<Edge>arg_edgelist) {
	nodes = n;
	edgelist = arg_edgelist;
	num_nodes = nodes.size();
	std::vector<int>().swap(parent);
	std::vector<int>().swap(rank);
	std::vector<Edge>().swap(mst);
}
int Graph::FindParent(int node) {
	if (node != parent[node])parent[node] = FindParent(parent[node]);//�ݹ�
	return parent[node];
}
void Graph::KruskalMST() {
	std::sort(edgelist.begin(), edgelist.end(), cmp);
	parent.resize(num_nodes);
	rank.resize(num_nodes);
	for (int i = 0; i < nodes.size(); ++i) {
		parent[i] = i;//��ʼʱÿ����ĸ��ڵ����Լ�
		rank[i] = 0;//��ʼʱÿ���ڵ��rank��0
	}
	for (auto ed : edgelist) {
		int root1 = FindParent(ed.src);
		int  root2 = FindParent(ed.dst);
		if (root1 != root2) {//root1���Ǹ��ڵ�
			mst.push_back(ed);
			if (rank[root1] < rank[root2]) {
				parent[root1] = root2;
				rank[root2] += 1;
			}
			else {
				parent[root2] = root1;
				rank[root1] += 1;
			}
		}
	}
	int cost = 0;
	for (auto ed : mst) {
		cost += ed.weight;
	}
}