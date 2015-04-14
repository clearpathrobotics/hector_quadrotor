#ifndef HECTOR_QUADROTOR_CONTROLLER_HELPERS_H
#define HECTOR_QUADROTOR_CONTROLLER_HELPERS_H

#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/TransformStamped.h"
#include "hector_uav_msgs/EnableMotors.h"
#include "std_msgs/Header.h"
#include "ros/ros.h"

namespace hector_quadrotor_controller
{

  class ImuSubscriberHelper
  {
  public:
    ImuSubscriberHelper(ros::NodeHandle &nh, std::string topic, sensor_msgs::Imu &imu)
        : imu_(imu)
    {
      imu_sub_ = nh.subscribe<sensor_msgs::Imu>(topic, 1, boost::bind(&ImuSubscriberHelper::imuCallback, this, _1));
    }

  private:
    sensor_msgs::Imu &imu_;
    ros::Subscriber imu_sub_;

    void imuCallback(const sensor_msgs::ImuConstPtr &imu)
    {
      imu_ = *imu;
    }

  };

  class EnableMotorsServiceHelper
  {
  public:
    EnableMotorsServiceHelper(ros::NodeHandle &nh, boost::function<bool(bool)> enable_motors)
        : enable_motors_(enable_motors)
    {
      motor_status_srv_ = nh.advertiseService("enable_motors", &EnableMotorsServiceHelper::enableMotorsCb, this);
    }

  private:
    ros::ServiceServer motor_status_srv_;
    boost::function<bool(bool)> enable_motors_;

    bool enableMotorsCb(hector_uav_msgs::EnableMotors::Request &req, hector_uav_msgs::EnableMotors::Response &res)
    {
      res.success = enable_motors_(req.enable);
      return true;
    }

  };

  // TODO switch to shapeshifter

  class OdomSubscriberHelper
  {
  public:
    OdomSubscriberHelper(ros::NodeHandle &nh, std::string topic, geometry_msgs::Pose &pose, geometry_msgs::Twist &twist,
                         geometry_msgs::Accel &acceleration, std_msgs::Header &header)
        : pose_(pose), twist_(twist), acceleration_(acceleration), header_(header)
    {
      odom_sub_ = nh.subscribe<nav_msgs::Odometry>(topic, 1,
                                                   boost::bind(&OdomSubscriberHelper::odomCallback, this, _1));
    }

  private:
    ros::Subscriber odom_sub_;

    geometry_msgs::Pose &pose_;
    geometry_msgs::Twist &twist_;
    geometry_msgs::Accel &acceleration_;
    std_msgs::Header header_;

    void odomCallback(const nav_msgs::OdometryConstPtr &odom)
    {
      // calculate acceleration
      if (!header_.stamp.isZero() && !odom->header.stamp.isZero())
      {
        const double acceleration_time_constant = 0.1;
        double dt((odom->header.stamp - header_.stamp).toSec());
        if (dt > 0.0)
        {
          acceleration_.linear.x =
              ((odom->twist.twist.linear.x - twist_.linear.x) + acceleration_time_constant * acceleration_.linear.x) /
              (dt + acceleration_time_constant);
          acceleration_.linear.y =
              ((odom->twist.twist.linear.y - twist_.linear.y) + acceleration_time_constant * acceleration_.linear.y) /
              (dt + acceleration_time_constant);
          acceleration_.linear.z =
              ((odom->twist.twist.linear.z - twist_.linear.z) + acceleration_time_constant * acceleration_.linear.z) /
              (dt + acceleration_time_constant);
          acceleration_.angular.x = ((odom->twist.twist.angular.x - twist_.angular.x) +
                                     acceleration_time_constant * acceleration_.angular.x) /
                                    (dt + acceleration_time_constant);
          acceleration_.angular.y = ((odom->twist.twist.angular.y - twist_.angular.y) +
                                     acceleration_time_constant * acceleration_.angular.y) /
                                    (dt + acceleration_time_constant);
          acceleration_.angular.z = ((odom->twist.twist.angular.z - twist_.angular.z) +
                                     acceleration_time_constant * acceleration_.angular.z) /
                                    (dt + acceleration_time_constant);
        }
      }

      header_ = odom->header;
      pose_ = odom->pose.pose;
      twist_ = odom->twist.twist;
    }

  };

  class PoseSubscriberHelper
  {
  public:
    PoseSubscriberHelper(ros::NodeHandle &nh, std::string topic, geometry_msgs::Pose &pose, geometry_msgs::Twist &twist,
                         geometry_msgs::Accel &acceleration, std_msgs::Header &header)
        : pose_(pose), twist_(twist), acceleration_(acceleration), header_(header)
    {
      pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(topic, 1,
                                                           boost::bind(&PoseSubscriberHelper::poseCallback, this,
                                                                       _1));
    }

  private:
    ros::Subscriber pose_sub_;

    geometry_msgs::Pose &pose_;
    geometry_msgs::Twist &twist_;
    geometry_msgs::Accel &acceleration_;
    std_msgs::Header header_;

    void poseCallback(const geometry_msgs::PoseStampedConstPtr &pose)
    {

      header_ = pose->header;
      pose_ = pose->pose;

      // TODO calculate twist and accel
//      twist_ = odom->twist.twist;
//      acceleration_ = ???
    }

  };

  class TransformSubscriberHelper
  {
  public:
    TransformSubscriberHelper(ros::NodeHandle &nh, std::string topic, geometry_msgs::Pose &pose,
                              geometry_msgs::Twist &twist, geometry_msgs::Accel &acceleration, std_msgs::Header &header)
        : pose_(pose), twist_(twist), acceleration_(acceleration), header_(header)
    {
      tf_sub_ = nh.subscribe<geometry_msgs::TransformStamped>(topic, 1,
                                                              boost::bind(&TransformSubscriberHelper::transformCallback,
                                                                          this, _1));
    }

  private:
    ros::Subscriber tf_sub_;

    geometry_msgs::Pose &pose_;
    geometry_msgs::Twist &twist_;
    geometry_msgs::Accel &acceleration_;
    std_msgs::Header header_;

    void transformCallback(const geometry_msgs::TransformStampedConstPtr &transform)
    {

      header_ = transform->header;
      pose_.position.x = transform->transform.translation.x;
      pose_.position.y = transform->transform.translation.y;
      pose_.position.z = transform->transform.translation.z;
      pose_.orientation = transform->transform.rotation;

      // TODO calculate twist and accel
//      twist_ = odom->twist.twist;
//      acceleration_ = ???
    }

  };


}

#endif //  HECTOR_QUADROTOR_CONTROLLER_HELPERS_H
