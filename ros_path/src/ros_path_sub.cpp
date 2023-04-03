#include <ros/ros.h> //导入ROS系统包含核心公共头文件 
#include<std_msgs/String.h>
/* 
定义回调函数chatterCallback，当收到chatter话题的消息就会调用这个函数。
消息通过boost shared_ptr（共享指针）来传递。 
但收到消息，通过ROS_INFO函数显示到终端 
*/ 
void chatterCallback(const std_msgs::String::ConstPtr& msg) 
{ 
    ROS_INFO("I heard: [%s]", msg->data.c_str()); 
} 
int main(int argc, char **argv) 
{ 
    ros::init(argc, argv, "listener"); //初始化ROS，指定节点名称为“listener” 
    ros::NodeHandle n; //实例化节点 
    ros::Subscriber sub = n.subscribe<std_msgs::String>("chatter", 1000, chatterCallback); 
    /* 
    定义订阅节点，名称为chatter，指定回调函数chatterCallback 
    但收到消息，则调用函数chatterCallback来处理。 
    参数1000，定义队列大小，如果处理不够快，超过1000，则丢弃旧的消息 
    */ 

     ros::spin(); //调用此函数才真正开始进入循环处理，直到 ros::ok()返回false才停 止。
     return 0; 
 }
