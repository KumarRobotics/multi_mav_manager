#include <multi_mav_manager/mav_manager_interface.h>

class MMcontrol;

MavManagerInterface::MavManagerInterface(std::string model_name, bool active, float battery_low, MMControl* mmc)
  : model_name_(model_name)
  , active_(active)
  , battery_low_(battery_low)
  , mmc_(mmc)
  , nh("multi_mav_services")
  , priv_nh("")
{
  //std::cout << "Creating Mav Manager Interface object for " << model_name_ << std::endl;

  if(active)
    std::cout << model_name_ << " is active" << std::endl;
  else
    std::cout << model_name_ << " is NOT active" << std::endl;

  //std::cout << "\tBattery low voltage is " << battery_low_ << std::endl;

  sc_motors = nh.serviceClient<std_srvs::SetBool>(               "/" + model_name_ + "/mav_services/motors");
  sc_takeoff = nh.serviceClient<std_srvs::Trigger>(           "/" + model_name_ + "/mav_services/takeoff");
  sc_goHome = nh.serviceClient<std_srvs::Trigger>(            "/" + model_name_ + "/mav_services/goHome");
  sc_goTo = nh.serviceClient<mav_manager::Vec4>(                 "/" + model_name_ + "/mav_services/goTo");
  sc_goToTimed = nh.serviceClient<mav_manager::GoalTimed>(       "/" + model_name_ + "/mav_services/goToTimed");
  sc_setDesVelInWorldFrame = nh.serviceClient<mav_manager::Vec4>("/" + model_name_ + "/mav_services/setDesVelInWorldFrame");
  sc_hover = nh.serviceClient<std_srvs::Trigger>(             "/" + model_name_ + "/mav_services/hover");
  sc_ehover = nh.serviceClient<std_srvs::Trigger>(            "/" + model_name_ + "/mav_services/ehover");
  sc_land = nh.serviceClient<std_srvs::Trigger>(              "/" + model_name_ + "/mav_services/land");
  sc_eland = nh.serviceClient<std_srvs::Trigger>(             "/" + model_name_ + "/mav_services/eland");
  sc_estop = nh.serviceClient<std_srvs::Trigger>(             "/" + model_name_ + "/mav_services/estop");

  odom_sub = nh.subscribe("/" + model_name_ + "/odom", 10, &MavManagerInterface::odom_cb, this);
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
  battery_ = msg.data;
  if(battery_ < battery_low_){
    std_srvs::Trigger srv;
    MavManagerInterface::deactivate();
  }
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
