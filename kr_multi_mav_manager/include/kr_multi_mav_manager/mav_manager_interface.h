#ifndef MAV_MANAGER_INTERFACE_H
#define MAV_MANAGER_INTERFACE_H

#include <Eigen/Core>
#include <string>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>

class MMControl;

class MavManagerInterface
{
  public:
    MavManagerInterface(std::string model_name, std::string odom_topic, std::string goto_base_name, bool active, float battery_low, MMControl* mmc);

    bool isActive() { return active_; }

    Eigen::Vector3f getPosition();

    ros::ServiceClient
      sc_motors_,
      sc_takeoff_,
      sc_goHome_,
      sc_goTo_,
      sc_goToTimed_,
      sc_setDesVelInWorldFrame_,
      sc_hover_,
      sc_ehover_,
      sc_land_,
      sc_eland_,
      sc_estop_;

    std::string model_name_;

  private:
    void odom_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void battery_cb(const std_msgs::Float32 &msg);

    bool deactivate_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool activate_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    void deactivate();

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_, battery_sub_;
    ros::ServiceServer srv_deactivate_, srv_activate_;
    nav_msgs::Odometry odom_;

    bool active_;
    float battery_;
    float battery_low_;
    MMControl* mmc_;
    Eigen::Vector3f position_;
};

#endif
