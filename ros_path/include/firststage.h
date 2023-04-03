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
	std::vector<double>vertex_x;//��������
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
	double per_ ;//ռ�İٷֱ�
	int subcol;
	int subrow;
	std::vector<double>subvertex_x;//�ĸ������xֵ
	std::vector<double>subvertex_y;//�ĸ������yֵ
	std::vector<Grid>result; //����submap�ĸ���
	std::vector<std::vector<Po>> resmatrix;
	// �����߶��ϵĵ�
	void PointLine(std::vector<Po*>& list, Po p0, Po p1);
	// �������θ��ǵĸ���
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
	void SetPercent(double s) { per = s; }//�趨���������˻��е�ռ��
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
	int part;//���ֳɼ���
	double length, width;
	double ggridlength;
	double ggridwidth;
	int col, row;
	GlobalMap(std::vector<double> a, std::vector<double> b,double l,double w,int num_drone);
	std::vector<SubMap> DivideMap(std::vector<Drone>& drone_vec);
	bool IsSegLine(double a1x, double a1y,double a2x,double a2y, double b1x, double b1y,double b2x,double b2y);//�ж�ֱ�����߶��Ƿ��ཻ��a1��a2��ֱ���ϵĵ������
	int Pos2Index(double x, double y);//�����ĸ�����
	void SetGridSizeandColRow(Drone_Vec drones_vec);//���������map��ʼ���ֿ�
	std::vector<double> Index2Pos(int i);//�����Է������ĵ�
};


