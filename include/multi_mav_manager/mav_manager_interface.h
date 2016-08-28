#ifndef MAV_MANAGER_INTERFACE_H
#define MAV_MANAGER_INTERFACE_H

#include <string>
#include <ros/ros.h>
#include <mav_manager/mav_manager_services.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
// #include <multi_mav_manager/mav_manager_interface.h>
// #include <std_srvs/Trigger.h>
// #include <std_srvs/SetBool.h>

class MMControl;

class MavManagerInterface
{
  private:

    bool active_;
    float battery_;
    float battery_low_;
    MMControl* mmc_;


    ros::NodeHandle nh, priv_nh; // TODO: do I need this here?

    void odom_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void battery_cb(const std_msgs::Float32 &msg);

    bool deactivate_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool activate_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    void deactivate();

  public:
 
    MavManagerInterface(std::string model_name, bool active, float battery_low, MMControl* mmc);

    std::string model_name_;
    bool isActive() { return active_; }

    // JT: I think these should be private members with public accessors
    nav_msgs::Odometry odom_;
    Eigen::Vector3f position_;
    Eigen::Vector3f goal_;

    // TODO: do I want offsets stored in here?

    ros::ServiceClient
      sc_motors,
      sc_takeoff,
      sc_goHome, sc_goTo,
      sc_goToTimed,
      sc_setDesVelInWorldFrame,
      sc_hover,
      sc_ehover,
      sc_land,
      sc_eland,
      sc_estop;

    ros::ServiceServer
      srv_deactivate,
      srv_activate;

    ros::Subscriber
      odom_sub,
      battery_sub;
};

#endif
