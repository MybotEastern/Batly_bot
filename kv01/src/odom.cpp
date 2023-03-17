#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float32.h"
#include <geometry_msgs/TransformStamped.h>

     
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.0;//0.1;
  double vy = 0.0;//-0.1;
  double vth = 0.0;//0.1;

void dist_callback(const geometry_msgs::Twist& dist)
{
        vx = dist.linear.x;
        vy = dist.linear.y;
        vth = dist.angular.z;
}
int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;
   
  ros::Subscriber subdist = n.subscribe("dist", 100, dist_callback);

  
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(20.0);
  while(n.ok()){

    ros::spinOnce();            // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
   
  
  
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
   
    //odom_trans.transform.rotation.x =odom_quat.x() ;
    //odom_trans.transform.rotation.y =odom_quat.y() ;
    //odom_trans.transform.rotation.z =odom_quat.z() ;
    //odom_trans.transform.rotation.w =odom_quat.w() ;
    
    

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    //odom.pose.pose.orientation.x = odom_quat.x() ;
    //odom.pose.pose.orientation.y = odom_quat.y() ;
    //odom.pose.pose.orientation.z = odom_quat.z() ;
    //odom.pose.pose.orientation.w = odom_quat.w() ;
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;

    //set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    odom.twist.covariance[0] = 0.001;
    odom.twist.covariance[7] = 0.001;
    odom.twist.covariance[35] = 0.001;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    
    r.sleep();
  }
}
