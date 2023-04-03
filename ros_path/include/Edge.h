#pragma once
#include<set>
#include<vector>
class Edge {
public:
	int src;
	int dst;
	int weight;
	Edge(int From, int To, int Weight) {
		this->src = From;
		this->dst = To;
		this->weight = Weight;
	};
	Edge operator=(const Edge& ed){
		src = ed.src;
		dst = ed.dst;
		weight = ed.weight;
		return *this;
	}
	bool operator==(const Edge& ed) const{
		if (src == ed.src &&dst == ed.dst && weight == ed.weight)return true;
		else return false;
	}
	bool operator<(const Edge& ed)const {
		if (weight != ed.weight)
			return weight < ed.weight;
		else if (src != ed.src)return src < ed.src;
		else if (dst != ed.dst)return dst < ed.dst;
	}
};
static bool cmp(Edge a, Edge b) {
	int aw = a.weight;
	int bw = b.weight;
	return (aw<bw);//升序排列
}
class Graph {
public:
	
	std::vector<std::set<int>>nodes;
	int num_nodes;
	std::vector<int>parent;
	std::vector<int>rank;
	std::vector<Edge>edgelist;
	std::vector<Edge>mst;//存放最小生成树的边
	Graph(std::vector<std::set<int>>n, std::vector<Edge>arg_edgelist);
	int FindParent(int node);
	
	void KruskalMST();
};