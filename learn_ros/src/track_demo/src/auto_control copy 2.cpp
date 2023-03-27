#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/PositionTarget.h>
#include <tf/tf.h>


geometry_msgs::Twist velocity;                  //接受键盘控制节点的速度
geometry_msgs::TwistStamped uav_velocity;       //无人机的当前速度
geometry_msgs::TwistStamped uav_velocity_send;  //发布的无人机的速度
geometry_msgs::PoseStamped uav_pose;            //无人机的当前位置

double roll = 0.0;                           
double pitch = 0.0;                           
double yaw = 0.0;                           

void uav_p_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    uav_pose = *msg;
    // tf::quaternionStampedMsgToTF(uav_pose.pose.orientation, quat);
    tf::Quaternion quat(uav_pose.pose.orientation.x,
        uav_pose.pose.orientation.y,
        uav_pose.pose.orientation.z,
        uav_pose.pose.orientation.w
    );
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}
void cmd_cb(const geometry_msgs::Twist::ConstPtr& msg) {
    velocity = *msg;
    uav_velocity_send.twist.linear.x = velocity.linear.x * cos(yaw) + velocity.linear.y * sin(yaw);
    uav_velocity_send.twist.linear.y = velocity.linear.x * sin(yaw) - velocity.linear.y * cos(yaw);
    uav_velocity_send.twist.linear.z = velocity.linear.z;
    uav_velocity_send.twist.angular.x = velocity.angular.x;
    uav_velocity_send.twist.angular.y = velocity.angular.y;
    uav_velocity_send.twist.angular.z = velocity.angular.z;
}

void uav_v_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    uav_velocity = *msg;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "auto_control");
    ros::NodeHandle nh;
    setlocale(LC_ALL, "");
    ros::Subscriber cmd_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, cmd_cb);
    // ros::Subscriber uav_v_sub = nh.subscribe<geometry_msgs::TwistStamped>
    //     ("/mavros/local_position/velocity_body", 10, uav_v_cb);
    ros::Publisher cmd_pub2px4 = nh.advertise<geometry_msgs::TwistStamped>
        ("/mavros/setpoint_velocity/cmd_vel", 10);

    // ros::Publisher cmd_pub2px4_linear = nh.advertise<mavros_msgs::PositionTarget>
    //     ("/mavros/setpoint_raw/local", 10);
    // ros::Publisher cmd_pub2px4_angular = nh.advertise<geometry_msgs::TwistStamped>
    //     ("/mavros/setpoint_velocity/cmd_vel", 10);

    // 订阅位置
    ros::Subscriber uav_p_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose", 10, uav_p_cb);

    ros::Rate rate(10);
    int n = 0;
    while (ros::ok())
    {
        cmd_pub2px4.publish(uav_velocity_send);
        if (n % 10 == 0) {
            ROS_INFO("vx:%.2f vy:%.2f vz:%.2f az:%.2f ",
                uav_velocity_send.twist.linear.x,
                uav_velocity_send.twist.linear.y,
                uav_velocity_send.twist.linear.z,
                uav_velocity_send.twist.angular.z
            );
        }
        // ROS_INFO("uav_vx:%.2f uav_vy:%.2f uav_vz:%.2f uav_az:%.2f ",
        //     uav_velocity.twist.linear.x,
        //     uav_velocity.twist.linear.y,
        //     uav_velocity.twist.linear.z,
        //     uav_velocity.twist.angular.z
        //     );
        ros::spinOnce();
        rate.sleep();
        n++;
    }
    return 0;
}