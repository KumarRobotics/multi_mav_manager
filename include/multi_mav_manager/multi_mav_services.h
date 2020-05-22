#ifndef MULTI_MAV_SERVICES_H
#define MULTI_MAV_SERVICES_H

#include <string>
#include <Eigen/Core>

#include <ros/ros.h>

#include <kr_mav_manager/mav_manager_services.h>
#include <multi_mav_manager/Formation.h>
#include <multi_mav_manager/RawPosFormation.h>
#include <multi_mav_manager/mav_manager_interface.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point32.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

class MMControl
{
  public:
    MMControl();
    void checkActiveRobots();

  private:
    bool motors_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool takeoff_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool goHome_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool goTo_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res);
    bool goToTimed_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res);

    bool goFormRawPos_cb(multi_mav_manager::RawPosFormation::Request &req, multi_mav_manager::RawPosFormation::Response &res);
    bool goFormLine_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res);
    bool goFormAngle_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res);
    bool goFormCircle_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res);
    bool goFormRect_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res);
    bool goFormGrid3d_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res);
    bool goFormTriangle_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res);

    void cleanFormation(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res);
    bool goForm(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res);
    bool checkCapt(multi_mav_manager::Formation::Response &res);
    void calculateDuration();
    bool calculateGoals();
    void createDistMatrix();

    bool setDesVelInWorldFrame_cb(kr_mav_manager::Vec4::Request &req, kr_mav_manager::Vec4::Response &res);
    bool hover_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool ehover_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool land_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool eland_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool estop_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    template <typename T>
      bool loop(const typename T::Request &req, typename T::Response &res,
                ros::ServiceClient MavManagerInterface::*sc, const std::string str);

    std::vector<std::shared_ptr<MavManagerInterface> > robots_;
    std::vector<std::shared_ptr<MavManagerInterface> > active_robots_;

    ros::NodeHandle nh_;
    std::vector<std::string> model_names_;
    std::vector<Eigen::Vector3f> formation_offsets_;
    std::vector<int> active_bots_;
    std::vector<nav_msgs::Odometry> odom_;

    std::vector<Eigen::Vector3f> activeRobotPositions();
    std::vector<Eigen::Vector3f> goals_;
    std::vector<int> assignment_matrix_;
    double capt_cost_;
    std::vector<double> distMatrixSquared_;
    double max_dist_;
    double duration_;
    double vmax_, amax_;
    ros::Time t_start_;

    enum class FormationShape {circle, rect, line, angle, grid3d, triangle};
    FormationShape formation_shape_;

    double formation_roll_;
    double formation_pitch_;
    double formation_yaw_;
    double formation_spacing_;
    std::string formation_message_;
    Eigen::Vector3f formation_center_;

    multi_mav_manager::Formation::Request f_req;
    multi_mav_manager::Formation::Response f_res;

    double rob_radius_;      // Safe radius of a robot
    double capt_spacing_;    // Minimum spacing required by capt to function
    double default_spacing_; // If spacing is unspecified or too small, make it a safe one
    //double minimum_height_;  // Minimum safety height above ground

    geometry_msgs::Point32 min_safety_bounds_; //Safety bounds for the formation
    geometry_msgs::Point32 max_safety_bounds_;

    int num_total_bots_;
    int num_active_bots_;

    ros::ServiceServer
      srv_motors_,
      srv_takeoff_,
      // srv_goHome_,
      srv_goTo_,
      srv_goToTimed_,
      srv_setDesVelInWorldFrame_,
      srv_hover_,
      srv_ehover_,
      srv_land_,
      srv_eland_,
      srv_estop_,
      srv_goFormRawPos_,
      srv_goFormCircle_,
      srv_goFormLine_,
      srv_goFormRect_,
      srv_goFormGrid3d_,
      srv_goFormAngle_,
      srv_goFormTriangle_;

};

#endif