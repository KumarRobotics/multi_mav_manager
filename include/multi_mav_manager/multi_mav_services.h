#ifndef MULTI_MAV_SERVICES_H
#define MULTI_MAV_SERVICES_H

#include <ros/ros.h>
#include <mav_manager/mav_manager_services.h>
#include <multi_mav_manager/Formation.h>
#include <multi_mav_manager/RawPosFormation.h>
#include <std_msgs/Float32.h>
#include <multi_mav_manager/mav_manager_interface.h>

class MMControl
{
  public:
    MMControl();
    void checkActiveRobots();

  protected:

  private:

    std::vector<std::shared_ptr<MavManagerInterface> > robots_;
    std::vector<std::shared_ptr<MavManagerInterface> > active_robots_;

    // JT: I don't think we need a priv_nh
    ros::NodeHandle nh, priv_nh;
    std::vector<std::string> model_names_;
    std::vector<Eigen::Vector3f> formation_offsets_;
    std::vector<int> active_bots_;
    std::vector<nav_msgs::Odometry> odom_;

    std::vector<Eigen::Vector3f> activeRobotPositions();
    std::vector<Eigen::Vector3f> goals_;
    int * assignment_matrix_;
    double capt_cost_;
    double * distMatrixSquared_;
    double max_dist_;
    double duration_;
    ros::Time t_start_;

    enum Shape {circle, rect, line, angle, grid3d};
    Shape formation_shape_;

    // JT: I suspect that these could be floats
    double formation_roll_;
    double formation_pitch_;
    double formation_yaw_;
    double formation_spacing_;
    std::string formation_message_;
    Eigen::Vector3f formation_center_;

    multi_mav_manager::Formation::Request f_req;
    multi_mav_manager::Formation::Response f_res;

    // JT: I suspect that these could be floats
    double rob_radius_ = 0.4;                                // Safe radius of a robot
    double capt_spacing_ = rob_radius_ * 2 * std::sqrt(2);   // Minimum spacing required by capt to function
    double default_spacing = 1.25 * capt_spacing_;           // If spacing is unspecified or too small, make it a safe one

    int num_total_bots;
    int num_active_bots;

    std::vector<ros::ServiceServer> srvs;

    ros::ServiceServer
      srv_motors_,
      srv_takeoff_,
      // srv_goHome_,
      srv_goTo_,
      srv_goToTimed,
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
      srv_goFormAngle_;

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

    void cleanFormation(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res);
    bool goForm(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res);
    bool checkCapt(multi_mav_manager::Formation::Response &res);
    void calculateDuration();
    void calculateGoals();
    void createDistMatrix();

    bool setDesVelInWorldFrame_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res);
    bool hover_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool ehover_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool land_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool eland_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool estop_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    template <typename T>
      bool loop(typename T::Request &req, typename T::Response &res, ros::ServiceClient MavManagerInterface::*sc, std::string str);
};

#endif
