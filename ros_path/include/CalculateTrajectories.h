#pragma once
#include<vector>
#include"Edge.h"
#include<iostream>
#include<algorithm>
class CalculateTrajectories {
public:
	int MAX_NODES;
	std::vector<std::vector<int>>PathSequence;
	int rows;
	int cols;
	std::vector<Edge>MSTvector;
	int MSTedges;
	std::set<Edge>allEdges;
	std::vector<std::set<int>>nodes;
	CalculateTrajectories(int r, int c, std::vector<Edge>MST);
	void InitializeGraph();
	void Add2AllEdges(std::set<Edge>&allEdges,int from, int to, int cost);
	void RemoveTheAppropriateEdges();
	void SafeRemoveEdge(Edge ed);
	bool SearchEdge(Edge ed);
	bool SearchNode(std::vector<std::set<int>>&nodes, int node);
	bool SearchNode(std::set<int>vec, int node);
	bool SearchNode(std::vector<int>vec, int node);
	bool CalculatePathsSequence(int startingnode);
};