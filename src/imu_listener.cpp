#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <math.h>
// void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
// {
//   ROS_INFO("Imu Seq: [%d]", msg->header.seq);
//   ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
// }

class subandpub
{
  public:
  ros::Subscriber sub;
  ros::Publisher pub;
  ros::NodeHandle n;
  bool flag;
  double roll_ref, pitch_ref, yaw_ref;

  subandpub()
  {
    sub = n.subscribe("/imu/data", 10, &subandpub::chatterCallback, this);
    pub = n.advertise<sensor_msgs::Imu>("/imu/use", 10);
    flag = false;
  }

  void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    // ROS_INFO("Imu Seq: [%d]", msg->header.seq);
    sensor_msgs::Imu new_msg;
    // ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    boost::array<double, 9> covar = msg->orientation_covariance;
    boost::array<double, 9> covar_acc = msg->orientation_covariance;
    boost::array<double, 9> covar_angv = msg->orientation_covariance;
    // geometry_msgs::Quaternion quat = msg->orientation;
    // tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    // tf::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);
    // // std::cout << "Roll: " << roll*180/M_PI << ", Pitch: " << pitch*180/M_PI << ", Yaw: " << yaw*180/M_PI << std::endl;
    // if(flag==false)
    // {
    //   roll_ref = roll;
    //   pitch_ref = pitch;
    //   yaw_ref = yaw;
    //   roll,pitch,yaw=0;
    //   flag = true;
    // }
    // else{
    //   roll-= roll_ref;
    //   pitch-= pitch_ref;
    //   yaw-= yaw_ref;
    //   if(yaw>360*M_PI/180) yaw -=  360*M_PI/180;
    //   if(yaw<-360*M_PI/180) yaw -=  360*M_PI/180;

    // }
    // tf::Quaternion pubq;
    // pubq.setRPY(roll, pitch,yaw);
    // geometry_msgs::Quaternion pubg;
    // tf::quaternionTFToMsg(pubq,pubg);
    // // std::cout << bla[0] << std::endl;
    covar[0] = 2*1e-5;
    covar[4] = 2*1e-5;
    covar[8] = 2*1e-5;
    covar_acc[0] = 1;
    covar_acc[4] = 1;
    covar_acc[8] = 1;
    covar_angv[0] = 1;
    covar_angv[4] = 1;
    covar_angv[8] = 1;
    new_msg = *msg;
    new_msg.orientation_covariance = covar;
    new_msg.angular_velocity_covariance = covar_angv;
    new_msg.linear_acceleration_covariance = covar_acc;
    new_msg.header.stamp = ros::Time::now();
    // new_msg.orientation = pubg;
    pub.publish(new_msg);
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_listener");
  ros::NodeHandle n;
  std::shared_ptr<subandpub> obj = std::make_shared<subandpub>();


  ros::spin();

  return 0;
}