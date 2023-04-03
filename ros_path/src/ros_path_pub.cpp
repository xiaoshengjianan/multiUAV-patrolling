#include"mcpp.h"
#include<ros/ros.h>
#include<std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Twist.h>//速度和力
#include<geometry_msgs/Pose.h>//位置
void odomcallback(){

}
int main(int argc,char** argv){
    Mcpp mc;
    ros::init(argc,argv,"node_pulisher");
    ros::NodeHandle nd;
    ros::Publisher pub=nd.advertise<std_msgs::String>("odometry_msg",100);
    ros::Rate loop_rate(10);

    return 0;
}


