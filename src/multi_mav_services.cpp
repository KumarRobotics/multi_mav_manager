#include <ros/ros.h>
#include <mav_manager/mav_manager_services.h>
#include <multi_mav_manager/Formation.h>
#include <multi_mav_manager/RawPosFormation.h>
#include <hungarian.h>

class MMControl
{
  public:
    MMControl();

  protected:


  private:
    ros::NodeHandle nh, priv_nh;
    std::vector<std::string> model_names_;
    std::vector<Eigen::Vector3f> formation_offsets_;
    Eigen::Vector3f formation_center_;
    std::vector<nav_msgs::Odometry> odom_;

    std::vector<Eigen::Vector3f> positions_;
    std::vector<Eigen::Vector3f> goals_;
    int * assignment_matrix_;
    double capt_cost_;
    double * distMatrixSquared_;
    double max_dist_;
    double duration_;
    ros::Time t_start_;

    double formation_roll_;
    double formation_pitch_;
    double formation_yaw_;

    double rob_radius_ = .4;                                 // Safe radius of a robot
    double capt_spacing_ = rob_radius_ * 2 * std::sqrt(2);   // Minimum spacing required by capt to function
    double default_spacing = 1.25 * capt_spacing_;           // If spacing is unspecified or too small, make it a safe one

    int num_bots;

    std::vector<ros::ServiceClient>
      sc_motors,
      sc_takeoff,
      sc_goHome,
      sc_goTo,
      sc_goToTimed, 
      sc_circle,
      sc_setDesVelInWorldFrame,
      sc_hover,
      sc_ehover,
      sc_land,
      sc_eland,
      sc_estop,
      sc_loadTraj,
      sc_prepTraj,
      sc_executeTraj;

    std::vector<ros::Subscriber> odom_subs;
    std::vector<ros::ServiceServer> srvs;

    bool motors_cb(mav_manager::Bool::Request &req, mav_manager::Bool::Response &res) {
      return loop<mav_manager::Bool>(req, res, sc_motors, "motors");
    }
    bool takeoff_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res) {
      return loop<mav_manager::Trigger>(req, res, sc_takeoff, "takeoff");
    }
    bool goHome_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res) {
      return loop<mav_manager::Trigger>(req, res, sc_goHome, "goHome");
    }

    bool goTo_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res)
    {
      // Since formation offsets are stored post rotation, in order to set an absolute rpy must remove stored offset
      req.roll    -= formation_roll_;
      req.pitch   -= formation_pitch_;
      req.yaw     -= formation_yaw_;

      return goForm(req, res);
    }
    bool goToTimed_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res)
    {
      return goForm(req, res);
    }
    bool goFormRawPos_cb(multi_mav_manager::RawPosFormation::Request &req, multi_mav_manager::RawPosFormation::Response &res){
      // Check equal length of formation_offsets_ and goTo service call
      if (req.goals.size() != (unsigned)num_bots)
      {
        // TODO handle case where goals are different length than robots better
        res.success = false;
        res.message = "req.goals.size() != num_bots";
        return true;
      }

      for(unsigned int i = 0; i < sc_goTo.size(); i++)
      {
        formation_offsets_[i][0] = req.goals[i].x;
        formation_offsets_[i][1] = req.goals[i].y;
        formation_offsets_[i][2] = req.goals[i].z;
        formation_offsets_[i][3] = req.goals[i].yaw;
      }

      multi_mav_manager::Formation::Request form_req;
      multi_mav_manager::Formation::Response form_res;
       
      goForm(form_req, form_res);
      res.success = form_res.success;
      res.message = form_res.message;

      return true;
    }

    bool goFormLine_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res)
    {
      cleanFormation(req, res);

      for(int i=0; i<num_bots; i++){
        formation_offsets_[i][0] = (i-((num_bots-1)/2.0)) * req.spacing;
        formation_offsets_[i][1] = 0.0;
        formation_offsets_[i][2] = 0.0;
      }

      return goForm(req, res);
    }

    bool goFormAngle_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res)
    {
      cleanFormation(req, res);
      
      if(req.param_names.size() != req.param_vals.size()){
        res.success = false;
        res.message += "\nParams list and vals are different sizes";
        return true;
      }

      double angle = M_PI/2; // Default angle is 90 degrees, ensure separation between second two

      for(unsigned int i=0; i < req.param_names.size(); i++){
        
        std::string str = req.param_names[i];
        double val = req.param_vals[i];

        if (str == "angle") 
          angle = val;
        else if (str == "angle_degrees")
          angle = val * (M_PI / 180);
        else 
            ROS_WARN_STREAM("Param " << str << " is invalid");
      }

      if(angle < M_PI/3){
        angle = M_PI/3;
        res.message += "\nDesired angle is too small, forced to minimum, pi/2 (90 degrees)";
      }

      int x_side = 1; // Right side
      int radius = 0; // Distance away from leader

      for(int i=0; i<num_bots; i++){
       formation_offsets_[i][0] = -1 * radius * std::cos(angle/2);
       formation_offsets_[i][1] = radius * x_side * std::sin(angle/2); 
       formation_offsets_[i][2] = 0;

       x_side = x_side * -1;
       if(x_side == -1)   radius += req.spacing;
      }

      return goForm(req, res);
    }

    bool goFormCircle_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res){
    
      cleanFormation(req, res);

      double radius = (num_bots * req.spacing) / (2 * M_PI);
   
      ROS_INFO_STREAM("spacing is: " << req.spacing);
      ROS_INFO_STREAM("radius is calculated to be: " << radius);
      // spc = 2 * pi * r / num_bots

      for(int i = 0; i < num_bots; i++)
      {
        // float theta_i = req.theta_start + i * theta_spacing;
        float theta_i = i * (2 * M_PI / num_bots);

        formation_offsets_[i](0) = radius * std::cos(theta_i);
        formation_offsets_[i](1) = radius * std::sin(theta_i);
        formation_offsets_[i](2) = 0;
        // formation_offsets_[i](3) = 0;
      }

      return goForm(req, res);
    }

    // TODO make pretty dividers for the sections

    bool goFormRect_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res){
      
      cleanFormation(req, res);

      int min_square_root = floor(sqrt(num_bots));
      int rows = min_square_root;
      int cols = 1;

      for(int rows = min_square_root; rows >=1; rows-=1){
        if(num_bots % rows == 0){
          cols = num_bots / rows;
          break;
        }
      }

      for(int i = 0; i < num_bots; i++)
      {
        int row_i = i / cols;
        int col_i = i % cols;

        formation_offsets_[i](0) = col_i * req.spacing - ((req.spacing * (cols-1)) / 2.0);
        formation_offsets_[i](1) = row_i * req.spacing - ((req.spacing * (rows-1)) / 2.0);
        formation_offsets_[i](2) = 0;
      }

      goForm(req, res);
      return true;
    }

    bool goFormGrid3d_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res){
     
      cleanFormation(req, res);
     
      int min_cube_root = floor(std::pow(num_bots, 1.0/3.0));

      int lays = min_cube_root; // Layers
      int rows = 1;
      int cols = 1;
      bool grid_found = false;

      while(!grid_found && lays >= 1)
      {
        if(num_bots % lays == 0){

          rows = floor(std::pow((num_bots / lays), 1.0/2.0));

          while(!grid_found && rows >= 1)
          {
            ROS_INFO_STREAM("trying rows = " << rows);
            if((num_bots / lays) % rows == 0){

              ROS_INFO_STREAM("found rows = " << rows);
              cols = (num_bots/lays) / rows;

              ROS_INFO_STREAM("found cols = " << cols);
              grid_found = true;
              break;
            }
            if(!grid_found) rows--;
          }
          break;
        }
        if(!grid_found) lays--;
      }

      ROS_INFO_STREAM("Grid3d found with Layers " << lays << " cols " << cols << " rows " << rows);

      for(int i = 0; i < num_bots; i++)
      {
        int lay_i = i / (rows * cols);
        int n_in_lay = i % (rows * cols);
        int row_i = n_in_lay / cols;
        int col_i = n_in_lay % cols;

        formation_offsets_[i](0) = col_i * req.spacing - ((req.spacing * (cols-1)) / 2.0);
        formation_offsets_[i](1) = row_i * req.spacing - ((req.spacing * (rows-1)) / 2.0);
        formation_offsets_[i](2) = lay_i * req.spacing - ((req.spacing * (lays-1)) / 2.0);
      }
      return goForm(req, res);
    }

    void cleanFormation(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res)
    {
      ROS_INFO_THROTTLE(1, "cleaning formation");
      if (req.spacing < capt_spacing_){
        res.message += "\nRequested spacing is too small, set to default: " + std::to_string(default_spacing);
        req.spacing = default_spacing;
      }
    }
    
    bool goForm(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res)
    {
      res.success = true;

      for(int i=0; i<3; i++)
        formation_center_(i) = req.center[i];

      formation_roll_ = req.roll;
      formation_pitch_ = req.pitch;
      formation_yaw_ = req.yaw;

      calculateGoals();       // Calculate global goal locations from formation information

      if(!checkCapt(res)){
        return true;            // Check that inital and goal locations are properly spaced
      }

      createDistMatrix();     // Calculated the distance matrix used by capt
      Hungarian h(assignment_matrix_, &capt_cost_, distMatrixSquared_, num_bots, goals_.size()); // Create hungarian object for capt
      h.computeAssignment();  // Use capt to find assignments
      calculateDuration();    // Determine temporal scaling based on max dist

      mav_manager::GoalTimed srv;

      for(int i=0; i<num_bots; i++){
      
        ROS_INFO_STREAM("Robot: " << i << " has goal number " << assignment_matrix_[i]);
        srv.request.goal[0] = goals_[assignment_matrix_[i]](0);
        srv.request.goal[1] = goals_[assignment_matrix_[i]](1);
        srv.request.goal[2] = goals_[assignment_matrix_[i]](2);
        srv.request.goal[3] = 0;  // TODO how to deal with individual robot yaw
        srv.request.duration = ros::Duration(duration_);
        srv.request.t_start = t_start_;
        
        // ROS_INFO_STREAM("service goes to " << srv.request.goal[0] << " with time " << srv.request.duration);
        
        if (!sc_goToTimed[i].call(srv))
        {
          res.message += "\nRobot " + std::to_string(i) + " failed during goForm";
          res.success = false;
        }
      }

      return true;
    }

    bool checkCapt(multi_mav_manager::Formation::Response &res){
      
      for(int i=0; i<num_bots; i++){
        for(int j=0; j<num_bots; j++)
        {
          if(i != j){ 
            if((positions_[i] - positions_[j]).norm() < capt_spacing_)
            {
              res.message += "\nPosition of robots " + std::to_string(i) + " and " + std::to_string(j) + " are too close for capt";
              res.success = false;
            }
            if((goals_[i] - goals_[j]).norm() < capt_spacing_)
            {
              res.message += "\nPosition of goals " + std::to_string(i) + " and " + std::to_string(j) + " are too close for capt";
              res.success = false;
            }
          }
        }
      }
      return res.success;
    }

    void calculateDuration(){
    
      max_dist_ = 0.0; 
      double dist_i = 0.0;
      for(int rob_i=0; rob_i<num_bots; rob_i++){

        dist_i = (positions_[rob_i] - goals_[assignment_matrix_[rob_i]]).norm();

        if(dist_i > max_dist_)  max_dist_ = dist_i;
      }

      duration_ = std::sqrt(max_dist_) * 2; // TODO make a better heuristic for temporal scaling

      t_start_ = ros::Time::now() + ros::Duration(num_bots * .01);  // Linearly scale start time by number of robots present
    }

    void calculateGoals(){

      // Define Rotation Matrices based on a yaw-pitch-roll euler angle rotation
      // Order is ZYX, yaw*pitch*roll

      Eigen::Matrix3f R_yaw;
      Eigen::Matrix3f R_pitch;
      Eigen::Matrix3f R_roll;

      R_yaw <<    std::cos(formation_yaw_),   std::sin(formation_yaw_), 0,
                  -std::sin(formation_yaw_),  std::cos(formation_yaw_), 0,
                  0,                          0,                        1;

      R_pitch <<  std::cos(formation_pitch_),   0,  -std::sin(formation_pitch_),
                  0,                            1,  0,
                  std::sin(formation_pitch_),   0,  std::cos(formation_pitch_);

      R_roll <<   1,  0,                            0,
                  0,  std::cos(formation_roll_),   -std::sin(formation_roll_),
                  0,  std::sin(formation_roll_),   std::cos(formation_roll_);


      for(int goal_i=0; goal_i<num_bots; goal_i++) {

        formation_offsets_[goal_i] = R_yaw * R_pitch * R_roll * formation_offsets_[goal_i];

        for(int dim_i=0; dim_i<3; dim_i++) {
          goals_[goal_i](dim_i) = 
            formation_center_[dim_i] + formation_offsets_[goal_i](dim_i);
        }

        if(goals_[goal_i][2] < 0) {
          ROS_INFO("Goal %d was too low at %2.2f so I set it to 0", goal_i, goals_[goal_i][2]);
          goals_[goal_i][2] = 0;   // Protect against robots crashing into the ground
                                                              // TODO make minimum height a param
                                                              // TODO make the boundaries of the space a param
        }
      }
    }

    void createDistMatrix(){
      capt_cost_ = 0;
      int num_goals = goals_.size();

      distMatrixSquared_ = new double[num_bots * num_goals];

      for(int robot_i = 0; robot_i < num_bots; robot_i++){
        for(int goal_i = 0; goal_i < num_goals; goal_i++){
          
          int q = (goal_i*num_goals) + robot_i;
          distMatrixSquared_[q] = 
            (positions_[robot_i] - goals_[goal_i]).squaredNorm();

        }
      }
    }

    // Callback for odometry updates from all robots
    void odom_cb(const nav_msgs::Odometry::ConstPtr &msg) {

      for(int i=0; i < num_bots; i++) {

        if ("/" + model_names_[i] == msg->child_frame_id)
        {
          odom_[i] = *msg;
          positions_[i](0) = msg->pose.pose.position.x;
          positions_[i](1) = msg->pose.pose.position.y;
          positions_[i](2) = msg->pose.pose.position.z;
          // ROS_INFO_STREAM("Got position for: " << i);
          return;
        }
      }

      ROS_INFO_STREAM("Failed to place an odometry messge");
      return;
    }

    bool setDesVelInWorldFrame_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res) {
      return loop<mav_manager::Vec4>(req, res, sc_setDesVelInWorldFrame, "setDesVelInWorldFrame");
    }
    bool hover_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res) {
      return loop<mav_manager::Trigger>(req, res, sc_hover, "hover");
    }
    bool ehover_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res) {
      return loop<mav_manager::Trigger>(req, res, sc_ehover, "ehover");
    }
    bool land_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res) {
      return loop<mav_manager::Trigger>(req, res, sc_land, "land");
    }
    bool eland_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res) {
      return loop<mav_manager::Trigger>(req, res, sc_eland, "eland");
    }
    bool estop_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res) {
      return loop<mav_manager::Trigger>(req, res, sc_estop, "estop");
    }
    bool loadTraj_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res) {
      return loop<mav_manager::Trigger>(req, res, sc_loadTraj, "loadTraj");
    }
    bool prepTraj_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res) {
      return loop<mav_manager::Vec4>(req, res, sc_prepTraj, "prepTraj");
    }
    bool executeTraj_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res) {
      return loop<mav_manager::Trigger>(req, res, sc_executeTraj, "executeTraj");
    }

    template <typename T>
    bool loop(typename T::Request &req, typename T::Response &res, std::vector<ros::ServiceClient> service_clients, std::string str)
    {
      T srv;
      srv.request = req;

      res.success = true;
      for(unsigned int i = 0; i < service_clients.size(); i++)
      {
        if (!service_clients[i].call(srv))
        {
          res.success = false;
          res.message = res.message + model_names_[i] + " failed to call " + str + ".\n";
        }
      }
      return true;
    }
};

MMControl::MMControl() : nh("multi_mav_services"), priv_nh("")
{
  ros::Duration(5.0).sleep();

  priv_nh.getParam("robot_radius", rob_radius_);

  capt_spacing_ = rob_radius_ * 2 * std::sqrt(2);   // Minimum spacing required by capt to function
  default_spacing = 1.25 * capt_spacing_;           // If spacing is unspecified or too small, make it a safe one

  std::cout << "Robot radius is set to " << rob_radius_ << std::endl;

  priv_nh.getParam("model_names", model_names_);
  num_bots = model_names_.size();

  std::cout << "Constructing multi_mav_control with " << num_bots << " bots" << std::endl;

  odom_.resize(num_bots); // Set odom size to number of robots // Set odom size to number of robots
  odom_subs.resize(num_bots);
  positions_.resize(num_bots);
  goals_.resize(num_bots);
  formation_offsets_.resize(num_bots);

  assignment_matrix_ = new int[num_bots];
  
  for(int i = 0; i < num_bots; i++)
  {
    std::cout << "Starting service clients for " << model_names_[i] << std::endl;
    sc_motors.push_back(nh.serviceClient<mav_manager::Bool>(               "/" + model_names_[i] + "/mav_services/motors"));
    sc_takeoff.push_back(nh.serviceClient<mav_manager::Trigger>(           "/" + model_names_[i] + "/mav_services/takeoff"));
    sc_goHome.push_back(nh.serviceClient<mav_manager::Trigger>(            "/" + model_names_[i] + "/mav_services/goHome"));
    sc_goTo.push_back(nh.serviceClient<mav_manager::Vec4>(                 "/" + model_names_[i] + "/mav_services/goTo"));
    sc_goToTimed.push_back(nh.serviceClient<mav_manager::GoalTimed>(       "/" + model_names_[i] + "/mav_services/goToTimed"));
    sc_setDesVelInWorldFrame.push_back(nh.serviceClient<mav_manager::Vec4>("/" + model_names_[i] + "/mav_services/setDesVelInWorldFrame"));
    sc_hover.push_back(nh.serviceClient<mav_manager::Trigger>(             "/" + model_names_[i] + "/mav_services/hover"));
    sc_ehover.push_back(nh.serviceClient<mav_manager::Trigger>(            "/" + model_names_[i] + "/mav_services/ehover"));
    sc_land.push_back(nh.serviceClient<mav_manager::Trigger>(              "/" + model_names_[i] + "/mav_services/land"));
    sc_eland.push_back(nh.serviceClient<mav_manager::Trigger>(             "/" + model_names_[i] + "/mav_services/eland"));
    sc_estop.push_back(nh.serviceClient<mav_manager::Trigger>(             "/" + model_names_[i] + "/mav_services/estop"));
    sc_loadTraj.push_back(nh.serviceClient<mav_manager::Trigger>(          "/" + model_names_[i] + "/mav_services/loadTraj"));
    sc_prepTraj.push_back(nh.serviceClient<mav_manager::Vec4>(             "/" + model_names_[i] + "/mav_services/prepTraj"));
    sc_executeTraj.push_back(nh.serviceClient<mav_manager::Trigger>(       "/" + model_names_[i] + "/mav_services/executeTraj"));


    // NOTE don't need service client for formation stuff. These are used to call services for individual robots

    // odom_subs.push_back(nh.subscribe<nav_msgs::Odometry>(                  "/" + model_names_[i] + "/odom",
    //       1, boost::bind(&odom_cb, _1, i)));

    //sub[i] = n.subscribe<sensor_msgs::Image>("/vrep/visionSensorData"+str_i, 1, boost::bind(&newImageTrigger_trgI, _1, i));
    
    std::vector<float> offsets;

    odom_subs[i] = nh.subscribe("/" + model_names_[i] + "/odom", 10, &MMControl::odom_cb, this);

    priv_nh.getParam("/" + model_names_[i] + "/formation_offsets", offsets);
    formation_offsets_[i][0] = offsets[0];
    formation_offsets_[i][1] = offsets[1];
    formation_offsets_[i][2] = offsets[2];
  }

  srvs.push_back(nh.advertiseService("motors", &MMControl::motors_cb, this));
  srvs.push_back(nh.advertiseService("takeoff", &MMControl::takeoff_cb, this));
  srvs.push_back(nh.advertiseService("goHome", &MMControl::goHome_cb, this));
  srvs.push_back(nh.advertiseService("goTo", &MMControl::goTo_cb, this));
  srvs.push_back(nh.advertiseService("goToTimed", &MMControl::goToTimed_cb, this));
  srvs.push_back(nh.advertiseService("setDesVelInWorldFrame", &MMControl::setDesVelInWorldFrame_cb, this));
  srvs.push_back(nh.advertiseService("hover", &MMControl::hover_cb, this));
  srvs.push_back(nh.advertiseService("ehover", &MMControl::ehover_cb, this));
  srvs.push_back(nh.advertiseService("land", &MMControl::land_cb, this));
  srvs.push_back(nh.advertiseService("eland", &MMControl::eland_cb, this));
  srvs.push_back(nh.advertiseService("estop", &MMControl::estop_cb, this));
  srvs.push_back(nh.advertiseService("loadTraj", &MMControl::loadTraj_cb, this));
  srvs.push_back(nh.advertiseService("prepTraj", &MMControl::prepTraj_cb, this));
  srvs.push_back(nh.advertiseService("executeTraj", &MMControl::executeTraj_cb, this));
  srvs.push_back(nh.advertiseService("goFormRawPos", &MMControl::goFormRawPos_cb, this));
  srvs.push_back(nh.advertiseService("goFormCircle", &MMControl::goFormCircle_cb, this));
  srvs.push_back(nh.advertiseService("goFormLine", &MMControl::goFormLine_cb, this));
  srvs.push_back(nh.advertiseService("goFormRect", &MMControl::goFormRect_cb, this));
  srvs.push_back(nh.advertiseService("goFormGrid3d", &MMControl::goFormGrid3d_cb, this));
  srvs.push_back(nh.advertiseService("goFormAngle", &MMControl::goFormAngle_cb, this));

  std::cout << "Multi Mav Manager is ready for action" << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_mav_services");

  MMControl multi_mav_control;

  ros::spin();

  return 0;
}
