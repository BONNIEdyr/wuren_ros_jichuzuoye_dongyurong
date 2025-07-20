#!/usr/bin/env python
import yaml
import rospy
#乌龟默认接收的消息类型是geometry_msgs.msg中的Twist
from geometry_msgs.msg import Twist
import os

if __name__ == "__main__":
    #获取当前功能包路径
    pkg_path = rospy.get_param('~package_path')
    #拼接完整路径
    yaml_path = os.path.join(pkg_path, 'config/ang_param.yaml')
    #加载yaml文件
    with open(yaml_path, 'r') as f:
        params = yaml.safe_load(f)
    #将参数加载到ROS参数服务器
    rospy.set_param('/turtle_ang_params', params) 
    #初始化节点并且命名
    rospy.init_node("turtle_vel_publisher", anonymous=True)
    #创建发布对象，话题名称为。。。
    pub = rospy.Publisher("/turtle_velocity", Twist, queue_size=10)
    #设置发布频率
    rate = rospy.Rate(10)
    #创建Twist消息对象
    vel_msg = Twist()
    
    #用msgshow命令查看Twist消息的结构发现是三位消息但是乌龟只能前进或者后退所以只需设置x分量即可

    vel_msg.linear.x = 1.0  
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    
    #以10Hz 频率持续发布速度指令，直到节点终止（Ctrl+C）
    while not rospy.is_shutdown():
        pub.publish(vel_msg)
        rate.sleep()