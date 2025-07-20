#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// 面向对象的代码风格
class Turtleism_control {
public:
  Turtleism_control() {
    // 创建句柄，一个节点可以有多个句柄，灵活工作，故句柄在类中创建，
    // 每个文件只能有一个main函数和节点一一对应，故节点在main函数中初始化
    ros::NodeHandle nh;
    
    // 从参数服务器中获取角速度，虽然上传的是twist消息但是只需要z方向的
    nh.getParam("/turtle_ang_params/turtle_velocity/angular/z", angular_vel);
    
    // 发布者发布乌龟的速度指令
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    
    // 订阅者订阅由.py脚本发布的速度指令
    sub_vel = nh.subscribe("/turtle_velocity", 10, &Turtleism_control::velocityCallback, this);
    
  }

  void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    geometry_msgs::Twist cmd_vel;
    
    // 使用订阅到的线速度
    cmd_vel.linear.x = msg->linear.x;
    cmd_vel.linear.y = msg->linear.y;
    cmd_vel.linear.z = msg->linear.z;
    
    // 使用预先存储的角速度
    cmd_vel.angular.z = angular_vel;

    // 输出调试信息
    ROS_INFO("Received velocity command: linear=(%.2f, %.2f, %.2f), angular=(%.2f)",
             cmd_vel.linear.x, cmd_vel.linear.y,
             cmd_vel.linear.z, cmd_vel.angular.z);
    // 在回调函数中发布速度指令，控制发布者在接受消息之后再发布消息
    cmd_pub.publish(cmd_vel);
  }

private:
  ros::Subscriber sub_vel; // 订阅者对象
  ros::Publisher cmd_pub;  // 发布者对象
  double angular_vel;      // 存储角速度的变量
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "turtle_controller"); // 初始化节点
  // 创建Turtleism_control类的实例
  Turtleism_control controller;
  ros::spin();
  return 0;
}
