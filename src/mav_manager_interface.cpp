#include <multi_mav_manager/mav_manager_interface.h>

#include <mav_manager/GoalTimed.h>
#include <mav_manager/Vec4.h>
#include <std_srvs/SetBool.h>

class MMcontrol;

MavManagerInterface::MavManagerInterface(std::string model_name, std::string odom_topic, bool active, float battery_low, MMControl* mmc)
  : model_name_(model_name)
  , active_(active)
  , battery_low_(battery_low)
  , mmc_(mmc)
  , nh("multi_mav_services")
{
  //std::cout << "Creating Mav Manager Interface object for " << model_name_ << std::endl;

  if(active)
    std::cout << model_name_ << " is active" << std::endl;
  else
    std::cout << model_name_ << " is NOT active" << std::endl;

  //std::cout << "\tBattery low voltage is " << battery_low_ << std::endl;

  const std::string service_base_name = "/" + model_name + "/mav_services/";
  sc_motors = nh.serviceClient<std_srvs::SetBool>(service_base_name + "motors");
  sc_takeoff = nh.serviceClient<std_srvs::Trigger>(service_base_name + "takeoff");
  sc_goHome = nh.serviceClient<std_srvs::Trigger>(service_base_name + "goHome");
  sc_goTo = nh.serviceClient<mav_manager::Vec4>(service_base_name + "goTo");
  sc_goToTimed = nh.serviceClient<mav_manager::GoalTimed>(service_base_name + "goToTimed");
  sc_setDesVelInWorldFrame = nh.serviceClient<mav_manager::Vec4>(service_base_name + "setDesVelInWorldFrame");
  sc_hover = nh.serviceClient<std_srvs::Trigger>(service_base_name + "hover");
  sc_ehover = nh.serviceClient<std_srvs::Trigger>(service_base_name + "ehover");
  sc_land = nh.serviceClient<std_srvs::Trigger>(service_base_name + "land");
  sc_eland = nh.serviceClient<std_srvs::Trigger>(service_base_name + "eland");
  sc_estop = nh.serviceClient<std_srvs::Trigger>(service_base_name + "estop");

  odom_sub = nh.subscribe("/" + model_name_ + "/" + odom_topic, 10, &MavManagerInterface::odom_cb, this);
  battery_sub = nh.subscribe("/" + model_name_ + "/battery", 10, &MavManagerInterface::battery_cb, this);

  srv_deactivate = nh.advertiseService("/" + model_name_ + "/deactivate", &MavManagerInterface::deactivate_cb, this);
  srv_activate = nh.advertiseService("/" + model_name_ + "/activate", &MavManagerInterface::activate_cb, this);

  position_ = Eigen::Vector3f::Zero();
  goal_ = Eigen::Vector3f::Zero();
}


void MavManagerInterface::odom_cb(const nav_msgs::Odometry::ConstPtr &msg) {
  odom_ = *msg;
  position_(0) = msg->pose.pose.position.x;
  position_(1) = msg->pose.pose.position.y;
  position_(2) = msg->pose.pose.position.z;
}

void MavManagerInterface::battery_cb(const std_msgs::Float32 &msg) {
  // See https://github.com/bitcraze/crazyflie-firmware/blob/master/src/hal/src/pm_f405.c
  // for crazyflie battery monitoring.

//   battery_ = msg.data;
//   ROS_INFO_STREAM(model_name_ << " battery reading "
//
//   unsigned int battery_update_rate = 10; // Hz
//   unsigned int timeout = 4; // seconds
//
//   static unsigned int counter(0);
//  if(battery_ < battery_low_) {
//    ROS_INFO_STREAM(model_name_ << "low battery reading " << counter << "/" << battery_update_rate * timeout);
//    counter++;
//    ROS_INFO_STREAM(model_name_ << "after low battery reading " << counter << "/" << battery_update_rate * timeout);
//  }
//  else {
//    ROS_INFO_STREAM(model_name_ << "low battery counter reset");
//    counter = 0;
//  }
//
//  // Disabled deactivate after low battery because shit went crazy
//  // Probably because deactivate isn't stable
//  counter = 0;
//
//  if (counter > timeout * battery_update_rate)
//  {
//    std_srvs::Trigger srv;
//    ROS_WARN_STREAM(model_name_ << " has a low battery of " << battery_ << "V.");
//    MavManagerInterface::deactivate();
//  }
}

bool MavManagerInterface::deactivate_cb(std_srvs::Trigger::Request &req,  std_srvs::Trigger::Response &res){
  MavManagerInterface::deactivate();
  res.success = true;
  return true;
}

void MavManagerInterface::deactivate(){
  ROS_INFO_STREAM(model_name_ << " is deactivating, it will land. Please turn off the motors when it is done landing" << std::endl);

  // TODO: add trackers monitoring to see if it is landed then turn off the motors
  // TODO: switch all of this over to actions
  // TODO: add a state machine to all of this

  //mmc_->checkActiveRobots(); // TODO: Figure out how to make this work
  std_srvs::Trigger srv;
  sc_land.call(srv);
  active_ = false;
}

bool MavManagerInterface::activate_cb(std_srvs::Trigger::Request &req,  std_srvs::Trigger::Response &res){
  ROS_INFO_STREAM(model_name_ << " is activating" << std::endl);

  // TODO: switch all of this over to actions
  // TODO: add a state machine to all of this

  // mmc_->checkActiveRobots() // Figure out how to make this work;
  active_ = true;
  return true;
}
