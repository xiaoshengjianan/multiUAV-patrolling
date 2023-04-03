#include"mcpp.h"
#define _CRT_SECURE_NO_WARNINGS
using namespace std;
 Mcpp::Mcpp() {
	vector<double>vertex_x{0,0,50,50};
	vector<double>vertex_y{0,50,50,0};
	double tempi = 0.0,tempj=0.0;
	int vertex_num = vertex_x.size();
	double l = 50;
	double w = 50;
	const int num_drone = 5;
	double v, h, theta, alpha_fv, alpha_fh, alpha_m;
	//std::allocator<Drone>alloc;
	//Drone* dronebuf = alloc.allocate(num_drone);
	//vector<Drone>drones(num_drone);
	GlobalMap globalmap(vertex_x,vertex_y,l,w,num_drone);
	Drone_Vec drones_vec("/home/jane/catkin_ws/src/rotors_simulator/mcpp/use.txt", num_drone);
	
	std::vector<SubMap>submaps=globalmap.DivideMap(drones_vec.drone_vec);
	PathPlanner path_plan(drones_vec);
	for (int i = 0; i < num_drone; ++i) {
		submaps[i].vertex_x = vertex_x;
		submaps[i].vertex_y = vertex_y;
	}
	std::vector<std::vector<Po>>().swap(path_all);//所有无人机的路径
	for (int i = 0; i < drones_vec.drone_vec.size(); ++i) {

			std::vector<Po> pt_tmp2;
			std::vector<std::vector<int>>pathi = path_plan.allpath[i];
			for (int j = 0; j < path_plan.allpath[i].size(); ++j) {
				//前一条边存在时
					if (j>1&&(path_plan.allpath[i][j][2] != path_plan.allpath[i][j - 1][0]) && (path_plan.allpath[i][j][3] != path_plan.allpath[i][j - 1][1])) {//��������ʱ��
						
						int r_head= path_plan.allpath[i][j-1][0];
						int c_head = path_plan.allpath[i][j - 1][1];
						Po p_head;
						double a = drones_vec.drone_vec[i].mymap.gridwidth / 2;
						double b = drones_vec.drone_vec[i].mymap.gridlength / 2;
						p_head.x= (drones_vec.drone_vec[i].mymap.resmatrix[r_head][c_head].x + b) * 1000;
						p_head.y= (drones_vec.drone_vec[i].mymap.resmatrix[r_head][c_head].y) * 1000;
						
						pt_tmp2.push_back(p_head);
						double angle_add = 3.1415926 / 120;//
						if (path_plan.allpath[i][j][3] > path_plan.allpath[i][j - 1][1]) {//向上拐弯
							double prim_x = (drones_vec.drone_vec[i].mymap.resmatrix[r_head][c_head].x + b) * 1000;//
							double prim_y = (drones_vec.drone_vec[i].mymap.resmatrix[r_head][c_head].y + a) * 1000;
							for (int n = 1; n <60; ++n) {
								Po p_tmp;
								p_tmp.y = prim_y - a * b / sqrt(b * b + a * a * tan(angle_add * n) * tan(angle_add * n));
								p_tmp.x = tan(angle_add * n)* ( - a * b / sqrt(b * b + a * a * tan(angle_add * n) * tan(angle_add * n)))+prim_x;
								pt_tmp2.push_back(p_tmp);
							}
							Po p_tail;
							p_tail.x = (drones_vec.drone_vec[i].mymap.resmatrix[r_head][c_head].x +2* b) * 1000;
							p_tail.y = (drones_vec.drone_vec[i].mymap.resmatrix[r_head][c_head].y+a) * 1000;
							pt_tmp2.push_back(p_tail);
						}
						else {//////////////////////////////////////////////////////////////////////////////////////////向下拐弯
							double prim_x = (drones_vec.drone_vec[i].mymap.resmatrix[r_head][c_head].x + b) * 1000;//
							double prim_y = (drones_vec.drone_vec[i].mymap.resmatrix[r_head][c_head].y - a) * 1000;
							for (int n = 1; n < 60; ++n) {
								Po p_tmp;
								
								p_tmp.y = prim_y +a * b / sqrt(b * b + a * a * tan(angle_add * n) * tan(angle_add * n));
								p_tmp.x = tan(angle_add * n) * a * b / sqrt(b * b + a * a * tan(angle_add * n) * tan(angle_add * n))+prim_x;
								pt_tmp2.push_back(p_tmp);
								
							}
							Po p_tail;
							p_tail.x = (drones_vec.drone_vec[i].mymap.resmatrix[r_head][c_head].x + 2 * b) * 1000;
							p_tail.y = (drones_vec.drone_vec[i].mymap.resmatrix[r_head][c_head].y-a) * 1000;
							pt_tmp2.push_back(p_tail);
						}
					}
				else {
				int r = path_plan.allpath[i][j][0];
				int c = path_plan.allpath[i][j][1];
				int index = path_plan.allpath[i][j][0] * drones_vec.drone_vec[i].mymap.subcol + (path_plan.allpath[i][j][1]);
				Po pi;
				/*pi.x = (drones_vec.drone_vec[i].mymap.result[index].pos_x)*20;
			   pi.y = (drones_vec.drone_vec[i].mymap.result[index].pos_y)*20;*/
				pi.x = (drones_vec.drone_vec[i].mymap.resmatrix[r][c].x) * 1000;
				pi.y = (drones_vec.drone_vec[i].mymap.resmatrix[r][c].y) * 1000;
				pt_tmp2.push_back(pi);
				}
			}
			path_all.push_back(pt_tmp2);
		}
	
	}
