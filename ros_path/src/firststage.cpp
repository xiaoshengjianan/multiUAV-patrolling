#include "firststage.h"
#define Drone_Name(i) DroneN##i
void SubMap::PointLine(std::vector<Po*>& list, Po p0, Po p1) {
	int dx = p1.x - p0.x;
	int dy = p1.y - p0.y;
	double Nx = gridlength;
	double Ny = gridwidth;
	double xStep ;
	double yStep ;
	int N =ceil( std::max<double>(abs(dx)/Nx, abs(dy)/Ny));//最大能分成几份
	if (dx == 0) {
		xStep = 0;
		yStep = Ny;
	}
	if (dy == 0) {
		xStep = Nx;
		yStep = 0;
	}
	else {
		xStep = Nx;
		yStep = Ny;
	}

	double x = p0.x>p1.x?p1.x:p0.x;
	double y = p0.y>p1.y?p1.y:p0.y;

	for (int step = 0; step <= N; step++) {//可能会和下一个有重合
		list.push_back(new Po(x,y));
		x += xStep;
		y += yStep;
	}//当矩形顶点是地图上的格子顶点时刚好得到的是格子线的切割点
}
std::vector<std::vector<Po>> SubMap::CoverPoint(int index,std::vector<Grid> &result, std::vector<double>vertex_x, std::vector<double>vertex_y)
{
	
	Po p0(vertex_x[0], vertex_y[0]);
	Po p1(vertex_x[1], vertex_y[1]);
	Po p2(vertex_x[2], vertex_y[2]);
	std::vector<Po> vertex{ p0, p1, p2 };
	//if (vertex_x.size() == 3) {
	//// pass	
	//}
	 if (vertex_x.size() == 4) {
		Po p3(vertex_x[3], vertex_y[3]);
		vertex.push_back(p3);
	}//暂时先考虑四个顶点的矩形子地图划分
	else {
		throw "Submap's points is not 4.";
	}
	 int count = 1;
	std::vector<Po*> list1;
	std::vector<Po*> list2;
	PointLine(list1, vertex[0], vertex[1]);
	PointLine(list2, vertex[1], vertex[2]);
	if (index == 0) {
		std::vector<std::vector<Po>> matrixmap(list2.size() - 1);
		for (int i = 0; i < list2.size() - 1; ++i) {
			matrixmap[i].resize(list1.size() - 1);
		}
		for (int i = 0; i < list2.size() - 1; ++i) {
			for (int j = 0; j < list1.size() - 1; ++j) {
				//double gridx =(list2[j]->x + list2[j + 1]->x)/2;//////////////////////
				//double gridy = (list1[i]->y + list1[i + 1]->y) / 2;////////////////////
				double gridy = (list2[i]->y + list2[i + 1]->y) / 2;//////////////////////
				double gridx = (list1[j]->x + list1[j + 1]->x) / 2;////////////////////
				Grid Gd(gridx, gridy, count);
				result.push_back(Gd);
				Po po(gridx, gridy);
				matrixmap[i][j] = (po);
				count += 1;
			}
		}//list2是算yi,list1是算xj
		subcol = list1.size() - 1;////////////////////////////////////////////////////////////
		subrow = list2.size() - 1;///////////////////////////////////////////////////////////////////

		Grid init = result[0];//默认从左上角开始出发,pathgrid是对result的重排
		if (subcol % 2 != 0) {//长和宽不是偶数的情况
			subcol = subcol + 1;
			for (int i = 0; i < matrixmap.size(); ++i) {
				double gridxplus = matrixmap[i].back().x + gridlength / 2.0;
				double gridyplus = matrixmap[i].back().y;
				Po po1(gridxplus, gridyplus);
				matrixmap[i].push_back(po1);
			}
		}
		if (subrow % 2 != 0) {
			subrow = subrow + 1;
			std::vector<Po>tmp(matrixmap[subrow-1].size());
			for (int i = 0; i < matrixmap[subrow-1].size(); ++i) {
				Po p(matrixmap[subrow - 1][i].x, matrixmap[subrow - 1][i].y + gridwidth / 2);
				tmp[i] = p;
			}
			matrixmap.push_back(tmp);
		}
		return matrixmap;
	}
	else 
	{
		std::vector<std::vector<Po>> matrixmap(list1.size() - 1);
		for (int i = 0; i < list1.size() - 1; ++i) {
			matrixmap[i].resize(list2.size() - 1);
		}
		for (int i = 0; i < list1.size() - 1; ++i) {
			for (int j = 0; j < list2.size() - 1; ++j) {
				//double gridx =(list2[j]->x + list2[j + 1]->x)/2;//////////////////////
				//double gridy = (list1[i]->y + list1[i + 1]->y) / 2;////////////////////
				double gridy = (list1[i]->y + list1[i + 1]->y) / 2;//////////////////////
				double gridx = (list2[j]->x + list2[j + 1]->x) / 2;////////////////////
				Grid Gd(gridx, gridy, count);
				result.push_back(Gd);
				Po po(gridx, gridy);
				matrixmap[i][j] = (po);
				count += 1;
			}
		}//list2是算xj,list1是算yi
		subcol = list2.size() - 1;////////////////////////////////////////////////////////////
		subrow = list1.size() - 1;///////////////////////////////////////////////////////////////////

		Grid init = result[0];//默认从左上角开始出发,pathgrid是对result的重排
		if (subcol % 2 != 0) {//长和宽不是偶数的情况
			
			for (int i = 0; i < matrixmap.size(); ++i) {
				double gridxplus = matrixmap[i].back().x + gridlength / 2.0;
				double gridyplus = matrixmap[i].back().y;
				Po po1(gridxplus, gridyplus);
				matrixmap[i].push_back(po1);
			}
			subcol = subcol + 1;
		}
		if (subrow % 2 != 0) {
			
			std::vector<Po>tmp(matrixmap[subrow-1].size());
			for (int i = 0; i < matrixmap[subrow-1].size(); ++i) {
				Po p(matrixmap[subrow-1][i].x, matrixmap[subrow-1][i].y + gridwidth / 2);
				tmp[i] = p;
			}
			matrixmap.push_back(tmp);
			subrow = subrow + 1;
		}
		return matrixmap;
	}
}

Drone::Drone(double velosity, double hight, double Theta, double Alpha_fv, double Alpha_fh,double Alpha_m) {
	v = velosity;
	h = hight;
	alpha_fv = Alpha_fv/180*3.14;
	alpha_fh = Alpha_fh/180*3.14;
	theta = Theta/180*3.14;
	alpha_m = Alpha_m/180*3.14;
	K_part = 2 * v * h * tan(alpha_fh / 2) / sin(alpha_m - theta + alpha_fv / 2);
	w_b = 2 * h * tan(alpha_fh / 2) / sin(alpha_m - theta + alpha_fv / 2);
	index = -1;
	per = 0;
	l_f = h * (1 / tan(alpha_m - theta - alpha_fv / 2) - 1 / tan(alpha_m - theta + alpha_fv / 2));
	mymap.gridlength = w_b;
	mymap.gridwidth = l_f;
}
GlobalMap::GlobalMap(std::vector<double> a, std::vector<double>b, double l, double w, int num_drone) {
	vertex_x = a;
	vertex_y = b;
	length = l;
	width = w;
	part = num_drone;
}
Drone_Vec::Drone_Vec(const char* filename, int num_drone) {
	std::fstream doc;
	doc.open(filename, std::ios::in);
	if (!doc.is_open()) {
		throw"Error openning file";
	}
	std::string s;
	double value[100];
	int tag = 0;
	double sum=0,percent=0;
	while (getline(doc, s)) {//读取字符串转化成数字	
		std::string s1 = "";
		for (int i = 0; i < s.length(); ++i) {
			if (s[i] == ' ')
			{
				value[tag] = atof(s1.c_str());
				s1 = "";
				tag += 1;
			}
			else {
				s1 += s[i];
			}
		}
		if (s1 != "")
			value[tag] = atof(s1.c_str());
		tag += 1;
	}
	for (int i = 1; i <= num_drone; ++i) {
		Drone Drone_Name(i)(value[6 * i - 6], value[6 * i - 5], value[6 * i - 4], value[6 * i - 3], value[6 * i - 2], value[6 * i - 1]);//动态取名并初始化
		drone_vec.push_back(Drone_Name(i));
		Drone_Name(i).AssignIndex(i);
		sum += Drone_Name(i).GetKPart();
	}
	for (int i = 0; i < num_drone; ++i) {
		percent = drone_vec[i].GetKPart() / sum;
		//drone_vec[i].SetPercent(percent);
		drone_vec[i].mymap.per_ = percent;
	}
	doc.close();
}

std::vector<SubMap> GlobalMap::DivideMap(std::vector<Drone>& drone_vec) {
	bool flag = false;
	double K = 0;
	double Ki = 0;
	double length = 0.0;
	int j = 0;
	int count = 0;//记录是不是第一个在非后临边上的点
	int c;
	double a_x=0.0, b_x=0.0, a_y=0.0, b_y=0.0, x_i=0.0, y_i=0.0, x_j=0.0, y_j=0.0;
	double A0=0.0, A1 = 0.0, A2 = 0.0;
	double B0 = 0.0, B1 = 0.0, B2 = 0.0;
	double C0 = 0.0, C1 = 0.0, C2 = 0.0,C11 = 0.0;
	std::vector<SubMap>Submaps(drone_vec.size());
	for (int i = 0; i < drone_vec.size(); ++i) {
		K += drone_vec[i].GetKPart();
		
	}
	for (int i = 0; i < drone_vec.size(); ++i) {
		Ki = drone_vec[i].GetKPart() / K;
		drone_vec[i].SetPercent(Ki);
		
	}
	//按最长边的方向划分，找出最长边，并记录最长边的临边，在临边上划分点,a是索引值更小的点
	for (int i = 0; i < vertex_x.size() - 1; ++i) {
		double l = sqrt((vertex_x[i] - vertex_x[i + 1]) * (vertex_x[i] - vertex_x[i + 1]) + (vertex_y[i] - vertex_y[i + 1]) * (vertex_y[i] - vertex_y[i + 1]));
		if (l > length) {
			j = i;
			length = l;
			int ii = (i + 1) % vertex_x.size();
			//向前的一条临边作为分割线
			a_x = vertex_x[(i + 1) % vertex_x.size()];
			b_x = vertex_x[(i + 2) % vertex_x.size()];
			a_y = vertex_y[(i + 1) % vertex_y.size()];
			b_y = vertex_y[(i + 2) % vertex_y.size()];
			//前临边表达式系数，A2,B2,C2
			if(a_x == b_x){
				A2 = 1;
				C2= -a_x;
				B2= 0;
				}
			else if (a_y = b_y) {
				A2 = 0;
				B2 = 1;
				C2 = -a_y;
			}
			else {
				A2 = 1;
				B2 = (a_x - b_x) / (a_y - b_y);
				C2 = -a_x + a_y * (b_x - a_x) / (a_y - b_y);
			}
			//记录后一条临边的参数
			if (i - 1 >=0) {
				if (vertex_x[i] == vertex_x[i - 1]) {
					A0 = 1;
					C0 = -vertex_x[i];
					B0 = 0;
				}
				else if (vertex_y[i] = vertex_y[i-1]) {
					A0 = 0;
					B0 = 1;
					C0 = -vertex_y[i];
				}
				else {
					A0= 1;
					B0 = (vertex_x[i-1] - vertex_x[i]) / (vertex_y[i] - vertex_y[i-1]);
					C0 = -vertex_x[i-1] + vertex_y[i-1] * (vertex_x[i] - vertex_x[i-1]) / (vertex_y[i] - vertex_y[i-1]);
				}
				
			}
			else {//i=0的情况
				if (vertex_x[i] == vertex_x[vertex_x.size()-1]) {
					A0 = 1;
					C0 = -vertex_x[i];
					B0 = 0;
				}
				else if (vertex_y[i] == vertex_y[vertex_y.size() - 1]) {
					A0= 0;
					B0 = 1;
					C0 = -vertex_y[i];
				}
				else {
					A0 = 1;
					B0 = (vertex_x[vertex_x.size() - 1] - vertex_x[i]) / (vertex_y[i] - vertex_y[vertex_y.size() - 1]);
					C0 = -vertex_x[vertex_x.size() - 1] + vertex_y[vertex_y.size() - 1] * (vertex_x[i] - vertex_x[vertex_x.size() - 1]) / (vertex_y[i] - vertex_y[vertex_y.size() - 1]);
				}
			}
			//记录本条最长边的参数 A1,B1,C1
			if (vertex_x[i] == vertex_x[ii]) {
				A1 = 1;
				C1 = -vertex_x[i];
				B1 = 0;
			  }
			else if (vertex_y[i] == vertex_y[ii]) {
				A1 = 0;
				C1 = -vertex_y[i];
				B1 = 1;
			}
			else {
				A1 = 1;
				B1 = (vertex_x[i] - vertex_x[ii]) / (vertex_y[ii] - vertex_x[i]);
				C1 = -vertex_x[i] + vertex_y[i] * (vertex_x[ii] - vertex_x[i]) / (vertex_y[ii] - vertex_y[i]);
			}
		}
	}
	double x_tmp, y_tmp;
	if (j >= 1)c = j - 1;//记录后临边的点
	else c = vertex_x.size() - 1;
	count = c;
	double calper = 0;
	//分割点初始化，在边上按比例线性插值，从靠近a的一端开始插值
	for (int i = 0; i < drone_vec.size()-1; ++i) {
		if (i == 0) {
			
			drone_vec[0].mymap.subvertex_x.push_back(vertex_x[j + 1]);
			drone_vec[0].mymap.subvertex_y.push_back(vertex_y[j + 1]);
			x_i = (drone_vec[0].GetPercent()) * (b_x - a_x) + a_x;
			y_i = (drone_vec[0].GetPercent()) * (b_y - a_y) + a_y;
		}
		else  {
			for (int m = 0; m <= i; ++m) {
				calper += drone_vec[m].GetPercent();
			}
			x_i = (calper) * (b_x - a_x) + a_x;
			y_i = (calper) * (b_y - a_y) + a_y;
			calper = 0;
		}
		
		drone_vec[i].mymap.subvertex_x.push_back(x_i);
		drone_vec[i].mymap.subvertex_y.push_back(y_i);
		drone_vec[i+1].mymap.subvertex_x.push_back(x_i);
		drone_vec[i+1].mymap.subvertex_y.push_back(y_i);
		C11 = -A1 * x_i - B1 * y_i;
		x_j = (B1 * C0 - B0 * C11) / (A1 * B0 - A0 * B1);
		y_j = (A0 * C11 - A1 * C0) / (A1 * B0 - A0 * B1);
		//求平行于最长边的直线与其前临边的交点
		if ((x_j - vertex_x[j]) * (x_j - vertex_x[c]) < 0) {
			//交点在后临边上
			x_j = x_j;
			y_j = y_j;
		}
		else {//交点不在后临边上，而是在其他的边
			flag = true;
			if (B1 == 0) {
				x_tmp = x_i;
				y_tmp = y_i + 10;
			}
			else if (A1 == 0) {
				y_tmp = y_i;
				x_tmp = x_i + 10;
			}
			else {
				x_tmp = x_i + 10;
				y_tmp = (-C11 - A1 * (x_tmp)) / B1;
			}
			for (int k = j + 2; k < vertex_x.size() - 1; ++k) {
				if (IsSegLine(x_i, y_i, x_tmp, y_tmp, vertex_x[k], vertex_y[k], vertex_x[(k + 1) % vertex_x.size()], vertex_y[(k + 1) % vertex_y.size()])){
					count =k;
					int kk=(k+1)%vertex_x.size();
					if (A1 == 0) {//分割线平行于y轴
						y_j=y_i;
						x_j=vertex_x[k]-(vertex_y[k]-y_j)*(vertex_x[k]-vertex_x[kk])/(vertex_y[k]- vertex_y[kk]);
					}
					else if (B1 == 0) {//分割线平行于x轴
						x_j=x_i;
						y_j=(vertex_y[k]- vertex_y[kk])*(vertex_x[k]-x_j)/(vertex_x[k]- vertex_x[kk]);
					}
					else if(vertex_x[k]== vertex_x[kk]) {//相交线平行于y轴
						x_j = vertex_x[k];
						y_j = (-C11 - A1 * x_j) / B1;
					}
					else if (vertex_y[k] == vertex_y[kk]) {//相交线平行于x轴
						x_j = vertex_y[k];
						y_j = (-C11 - B1 * x_j) / A1;
					}
					else {//两线都有斜率
						double k_g=(vertex_y[k]- vertex_y[kk])/(vertex_x[k]- vertex_x[kk]);
						double b_g= vertex_y[k]-k_g* vertex_x[k];
						x_j = -(B1 * b_g + C11) / (k_g * B1 + A1);
						y_j = b_g + k_g * vertex_x[k];
					}
				}
			}
		}
		drone_vec[i].mymap.subvertex_x.push_back(x_j);
		drone_vec[i].mymap.subvertex_y.push_back(y_j);
		drone_vec[i+1].mymap.subvertex_x.push_back(x_j);
		drone_vec[i+1].mymap.subvertex_y.push_back(y_j);
		if (count !=c) {//后临边上的端点属于某子地图
			drone_vec[i].mymap.subvertex_x.push_back(vertex_x[count+1]);
			drone_vec[i].mymap.subvertex_y.push_back(vertex_y[count+1]);
				c = count;
			}
		if (i == 0) {
			drone_vec[0].mymap.subvertex_x.push_back(vertex_x[j]);
			drone_vec[0].mymap.subvertex_y.push_back(vertex_y[j]);
		}
		double tmp = drone_vec[0].mymap.subvertex_x[1];
		drone_vec[0].mymap.subvertex_x[1] = drone_vec[0].mymap.subvertex_x[3];
		drone_vec[0].mymap.subvertex_x[3] = tmp;//交换第一块子地图的顶点使得与其他地图统一
		if (i == drone_vec.size()-2) {
			
			drone_vec[i+1].mymap.subvertex_x.push_back(vertex_x[(j + 2) % (drone_vec.size())]);
			drone_vec[i + 1].mymap.subvertex_y.push_back(vertex_y[(j + 2) % (drone_vec.size())]);
			if (!flag) {
				drone_vec[i + 1].mymap.subvertex_x.push_back(vertex_x[(j + 3) % (drone_vec.size())]);
				drone_vec[i + 1].mymap.subvertex_y.push_back(vertex_y[(j + 3) % (drone_vec.size())]);
			}
			i = i + 1;
		}
	}//调整第二块小地图以及之后的各顶点排序
	
	for (int i = 1; i < drone_vec.size(); ++i) {
		std::reverse(drone_vec[i].mymap.subvertex_x.begin()+2, drone_vec[i].mymap.subvertex_x.end());
		std::reverse(drone_vec[i].mymap.subvertex_y.begin()+2, drone_vec[i].mymap.subvertex_y.end());
	}
	for (int i = 0; i < drone_vec.size(); ++i) {
		//记录子地图的小格子。
		drone_vec[i].mymap.resmatrix=drone_vec[i].mymap.CoverPoint(i,drone_vec[i].mymap.result, drone_vec[i].mymap.subvertex_x, drone_vec[i].mymap.subvertex_y);
		//drone_vec[i].mymap.subrow = drone_vec[i].mymap.resmatrix.size();/////////////////////////////////////////////
		//drone_vec[i].mymap.subcol = drone_vec[i].mymap.resmatrix[0].size();////////////////////////////////////////////
		
	}
	for (int i = 0; i < drone_vec.size(); ++i) {
		Submaps[i] = drone_vec[i].mymap;
	}
	return Submaps;
}
bool GlobalMap::IsSegLine(double a1x, double a1y, double a2x, double a2y, double b1x, double b1y, double b2x, double b2y) {
	Eigen::Vector3d a2a1(a1x - a2x, a1y - a2y,0);
	Eigen::Vector3d a1b1(b1x - a1x, b1y - a1y,0);
	Eigen::Vector3d a1b2(b2x - a1x, b2y - a1y,0);
	double res=0;
	Eigen::Vector3d a2a1a1b1 = a2a1.cross(a1b1);//叉乘在二维中得到的向量在z轴上，eigen只能用三维向量
	Eigen::Vector3d a2a1a1b2 = a2a1.cross(a1b2);
	res = a2a1a1b1.dot(a2a1a1b2);
	//res = (a2a1.cross(a1b1)).dot(a2a1.cross(a1b2));
	if (res < 0)return true;
	else return false;
}
void GlobalMap::SetGridSizeandColRow(Drone_Vec drones_vec) {
	double w_bmin = drones_vec.drone_vec[0].GetW_b();
	double l_fmin = drones_vec.drone_vec[0].GetL_f();
	double w_bi, l_fi;
	for (int i = 0; i < drones_vec.drone_vec.size(); ++i) {
		w_bi = drones_vec.drone_vec[i].GetW_b();
		l_fi = drones_vec.drone_vec[i].GetL_f();
		if (w_bi < w_bmin) {
			w_bmin = w_bi;
		}
		if (l_fi < l_fmin) {
			l_fmin = l_fi;
		}
	}
	if (w_bmin > l_fmin) {
		ggridlength = ggridwidth = l_fmin;
	}
	else {
		ggridlength = ggridwidth = w_bmin;
	}
	if (length / ggridlength > floor(length / ggridlength)) {
		rescol = true;
		col = floor(length / ggridlength) + 1;
	}
	if (width / ggridwidth > floor(width / ggridwidth)) {
		resrow = true;
		row = floor(width / ggridwidth) + 1;
	}
}
int GlobalMap::Pos2Index(double x, double y) {
	int col_tmp; 
	int row_tmp; 
	int index;
	col_tmp= ceil((x - vertex_x[0]) / ggridlength) ;
	row_tmp= ceil((y - vertex_y[0]) / ggridwidth);
	if (col_tmp > col || row_tmp > row||col_tmp<0||row_tmp<0) {//取点错误情况
		std::cout << "point is out of map." << std::endl;
		exit(0);
	}
	index = (row_tmp - 1) * col + col_tmp;
	return index;
}
std::vector<double> GlobalMap::Index2Pos(int i) {
	std::vector<double>pos;
	int col_tmp, row_tmp;
	double x,y;
	if (i <= 0 || i > (col * row)) {
		std::cout << "point is out of map." << std::endl;
		exit(0);
	}
	col_tmp = i % col;
	row_tmp = i / col;
	x = vertex_x[0] + (col_tmp-1) * ggridlength+ggridlength/2;
	pos.push_back(x);
	y = vertex_y[0] + (row - row_tmp) * ggridwidth + ggridwidth / 2;
	pos.push_back(y);
	return pos;
}