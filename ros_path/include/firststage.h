#pragma once
#include<iostream>
#include <vector>
#include<cmath>
#include<math.h>
#include<Eigen/Dense>
#include <iostream>
#include<fstream>
#include<string>
#include<algorithm> 
#include<map>
#include<exception>
struct Point
{
	int x;
	int y;
	Point() {};
	Point(int x_, int y_) : x(x_), y(y_) {};
};

class Drone;
class Drone_Vec;
class Grid {
public:
	int index;
	double pos_x, pos_y;
	Grid(double x_,double y_,int in):pos_x(x_),pos_y(y_),index(in) {};
};
class Map {
public:
	
	bool rescol;
	bool resrow;
	std::vector<double>vertex_x;//顶点容器
	std::vector<double>vertex_y;
	Map() :rescol(false), resrow(false){};
};
struct Po
{
	double x;
	double y;
	Po() {};
	Po(double x_, double y_) : x(x_), y(y_) {};
};
class SubMap:public Map {
public:
	double gridlength;
	double gridwidth;
	int index;
	double per_ ;//占的百分比
	int subcol;
	int subrow;
	std::vector<double>subvertex_x;//四个顶点的x值
	std::vector<double>subvertex_y;//四个顶点的y值
	std::vector<Grid>result; //属于submap的格子
	std::vector<std::vector<Po>> resmatrix;
	// 计算线段上的点
	void PointLine(std::vector<Po*>& list, Po p0, Po p1);
	// 计算多边形覆盖的格子
	std::vector<std::vector<Po>> CoverPoint(int index, std::vector<Grid>& result, std::vector<double>vertex_x, std::vector<double>vertex_y);
};
class Drone {
public:
	double v;
	double h, alpha_fv, alpha_fh, l_f, theta, alpha_m,w_b;
	double K_part;
	double per;
	SubMap mymap;
	int index;
	Drone(double velosity, double hight, double Theta, double Alpha_fv, double Alpha_fh,  double Alpha_m);
	void AssignIndex(int i) { index = i; }
	void SetPercent(double s) { per = s; }//设定在所有无人机中的占比
	double GetKPart() { return K_part; }
	double GetPercent() { return per; }
	double GetL_f() { return l_f; }
	double GetW_b() { return w_b;}
	void SetSubMap(SubMap submap) { mymap = submap; }
	SubMap GetMyMap() { return mymap; }
};
class Drone_Vec {
public:
	std::vector<Drone> drone_vec;
	Drone_Vec(const char* filename, int num_drone);
	
};
class GlobalMap:public Map {
public:
	int part;//被分成几块
	double length, width;
	double ggridlength;
	double ggridwidth;
	int col, row;
	GlobalMap(std::vector<double> a, std::vector<double> b,double l,double w,int num_drone);
	std::vector<SubMap> DivideMap(std::vector<Drone>& drone_vec);
	bool IsSegLine(double a1x, double a1y,double a2x,double a2y, double b1x, double b1y,double b2x,double b2y);//判断直线与线段是否相交，a1、a2是直线上的点的坐标
	int Pos2Index(double x, double y);//属于哪个格子
	void SetGridSizeandColRow(Drone_Vec drones_vec);//计算格子与map初始化分开
	std::vector<double> Index2Pos(int i);//仅可以返回中心点
};


