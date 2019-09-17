#include <multi_mav_manager/multi_mav_services.h>

#include <hungarian.h>

bool MMControl::motors_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
  return loop<std_srvs::SetBool>(req, res, &MavManagerInterface::sc_motors_, "motors");
}

bool MMControl::takeoff_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  return loop<std_srvs::Trigger>(req, res, &MavManagerInterface::sc_takeoff_, "takeoff");
}

/*
bool MMControl::goHome_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  // JT: This should use the home location for each robot and leverage CAPT to go there. Otherwise, there could be collisions
  return loop<std_srvs::Trigger>(req, res, &MavManagerInterface::sc_goHome, "goHome");
}
*/

bool MMControl::goTo_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res)
{
  // Since formation offsets are stored post rotation, in order to set an absolute rpy must remove stored offset
  req.roll    -= formation_roll_;
  req.pitch   -= formation_pitch_;
  req.yaw     -= formation_yaw_;

  return goForm(req, res);
}

bool MMControl::goToTimed_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res)
{
  return goForm(req, res);
}

bool MMControl::goFormRawPos_cb(multi_mav_manager::RawPosFormation::Request &req, multi_mav_manager::RawPosFormation::Response &res){
  // Check equal length of formation_offsets_ and goTo service call
  if (req.goals.size() != (unsigned)num_active_bots_)
  {
    // TODO handle case where goals are different length than robots better
    res.success = false;
    res.message = "req.goals.size() != num_active_bots";
    return true;
  }

  // TODO: change this sizing thing
  for(int i = 0; i < num_active_bots_; i++)
  {
    formation_offsets_[i][0] = req.goals[i].x;
    formation_offsets_[i][1] = req.goals[i].y;
    formation_offsets_[i][2] = req.goals[i].z;
  }

  multi_mav_manager::Formation::Request form_req;
  multi_mav_manager::Formation::Response form_res;

  goForm(form_req, form_res);
  res.success = form_res.success;
  res.message = form_res.message;

  return true;
}

bool MMControl::goFormLine_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res)
{
  formation_shape_ = FormationShape::line;
  cleanFormation(req, res);

  for(int i=0; i<num_active_bots_; i++){
    formation_offsets_[i][0] = (i-((num_active_bots_-1)/2.0)) * req.spacing;
    formation_offsets_[i][1] = 0.0;
    formation_offsets_[i][2] = 0.0;
  }

  return goForm(req, res);
}

bool MMControl::goFormTriangle_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res)
{
  formation_shape_ = FormationShape::triangle;
  cleanFormation(req, res);

  double R = req.spacing;
  double th = M_PI / 3.0;

  if(num_active_bots_ == 6){
    formation_offsets_[0][0] = R *  1.0 * std::sin(th); formation_offsets_[0][1] = 0.0;                     formation_offsets_[0][2] = 0.0;
    formation_offsets_[1][0] = R * -1.0 * std::sin(th); formation_offsets_[1][1] = 0.0;                     formation_offsets_[1][2] = 0.0;
    formation_offsets_[2][0] =                     0.0; formation_offsets_[2][1] = R *  1.0 * std::cos(th); formation_offsets_[2][2] = 0.0;
    formation_offsets_[3][0] =                     0.0; formation_offsets_[3][1] = R * -1.0 * std::cos(th); formation_offsets_[3][2] = 0.0;
    formation_offsets_[4][0] = R * -1.0 * std::sin(th); formation_offsets_[4][1] = R *  2.0 * std::cos(th); formation_offsets_[4][2] = 0.0;
    formation_offsets_[5][0] = R * -1.0 * std::sin(th); formation_offsets_[5][1] = R * -2.0 * std::cos(th); formation_offsets_[5][2] = 0.0;

    // for(int i = 0; i <=5; i++){
    //   ROS_INFO_STREAM("point " << i << " (" << formation_offsets_[i][0] << ", "<< formation_offsets_[i][1] << ", "<< formation_offsets_[i][2] << ")");
    // }

  } else {
    res.success = false;
    res.message = "Triangle only defined for 6 robots right now";
  }
  return goForm(req, res);
}

bool MMControl::goFormAngle_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res)
{
  formation_shape_ = FormationShape::angle;
  cleanFormation(req, res);

  if(req.param_names.size() != req.param_vals.size()){
    res.success = false;
    res.message += "\nParams list and vals are different sizes";
    return true;
  }

  double angle = M_PI/3; // Default angle is 90 degrees, ensure separation between second two

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

  double x_side = 1.0; // Right side
  double radius = 0.0; // Distance away from leader

  for(int i=0; i<num_active_bots_; i++){
    formation_offsets_[i][0] = -1 * radius * std::cos(angle/2);
    formation_offsets_[i][1] = radius * x_side * std::sin(angle/2);
    formation_offsets_[i][2] = 0;

    x_side = x_side * -1;
    if(x_side == -1)   radius += req.spacing;
  }

  return goForm(req, res);
}

bool MMControl::goFormCircle_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res){

  formation_shape_ = FormationShape::circle;
  cleanFormation(req, res);

  double radius = (num_active_bots_ * req.spacing) / (2 * M_PI);

  for(int i = 0; i < num_active_bots_; i++)
  {
    float theta_i = i * (2 * M_PI / num_active_bots_);

    formation_offsets_[i](0) = radius * std::cos(theta_i);
    formation_offsets_[i](1) = radius * std::sin(theta_i);
    formation_offsets_[i](2) = 0;
  }

  return goForm(req, res);
}

bool MMControl::goFormRect_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res){

  formation_shape_ = FormationShape::rect;
  cleanFormation(req, res);

  int min_square_root = floor(sqrt(num_active_bots_));
  int rows = min_square_root;
  int cols = 1;

  for(int rows = min_square_root; rows >=1; rows-=1){
    if(num_active_bots_ % rows == 0){
      cols = num_active_bots_ / rows;
      break;
    }
  }

  for(int i = 0; i < num_active_bots_; i++)
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

bool MMControl::goFormGrid3d_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res){

  formation_shape_ = FormationShape::grid3d;
  cleanFormation(req, res);

  int min_cube_root = floor(std::pow(num_active_bots_, 1.0/3.0));

  int lays = min_cube_root; // Layers
  int rows = 1;
  int cols = 1;
  bool grid_found = false;

  while(!grid_found && lays >= 1)
  {
    if(num_active_bots_ % lays == 0){

      rows = floor(std::pow((num_active_bots_ / lays), 1.0/2.0));

      while(!grid_found && rows >= 1)
      {
        ROS_INFO_STREAM("trying rows = " << rows);
        if((num_active_bots_ / lays) % rows == 0){

          ROS_INFO_STREAM("found rows = " << rows);
          cols = (num_active_bots_/lays) / rows;

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

  for(int i = 0; i < num_active_bots_; i++)
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

void MMControl::cleanFormation(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res)
{
  //checkActiveRobots();
  ROS_INFO_THROTTLE(1, "cleaning formation");

  f_res.message = "";
  f_res.success = true;

  if (req.spacing < capt_spacing_){
    res.message += "\nRequested spacing is too small, set to default: " + std::to_string(default_spacing_);
    req.spacing = default_spacing_;
  }

  f_req.spacing = req.spacing;
  f_req.center = req.center;
  f_req.roll = req.roll;
  f_req.pitch = req.pitch;
  f_req.yaw = req.yaw;
}

bool MMControl::goForm(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res)
{
  res.success = true;

  for(int i=0; i<3; i++)
    formation_center_(i) = req.center[i];

  //Check if formation center is within safety bounds
  if( ((formation_center_(0) < min_safety_bounds_.x) || (formation_center_(0) > max_safety_bounds_.x)) ||
      ((formation_center_(1) < min_safety_bounds_.y) || (formation_center_(1) > max_safety_bounds_.y)) ||
      ((formation_center_(2) < min_safety_bounds_.z) || (formation_center_(2) > max_safety_bounds_.z)) )
  {
    ROS_ERROR("Formation center (%2.2f  %2.2f %2.2f) is outside saftey bounds (%2.2f  %2.2f) (%2.2f  %2.2f) (%2.2f  %2.2f)",
        formation_center_(0), formation_center_(1), formation_center_(2),
        min_safety_bounds_.x, max_safety_bounds_.x, min_safety_bounds_.y, max_safety_bounds_.y, min_safety_bounds_.z, max_safety_bounds_.z);
    res.message += "Formation center is outside saftey bounds";
    res.success = false;
    return false;
  }

  formation_roll_ = req.roll;
  formation_pitch_ = req.pitch;
  formation_yaw_ = req.yaw;

  if(!calculateGoals()){       // Calculate global goal locations from formation information
    ROS_ERROR("Goals computed outside saftey bounds, check formation bounds and center");
    res.message += "failed during calculate Goals";
    res.success = false;
    return false;
  }
  ROS_INFO("Successfully calcuated goals.");

  if(!checkCapt(res)){
    ROS_ERROR("Issue here? Check that initial and goal locations are properly spaced");
    return false;            // Check that initial and goal locations are properly spaced
  }

  ROS_INFO("CAPT...");
  createDistMatrix();     // Calculated the distance matrix used by capt
  Hungarian h(assignment_matrix_.data(), &capt_cost_, distMatrixSquared_.data(),
              num_active_bots_, goals_.size()); // Create hungarian object for capt
  h.computeAssignment();  // Use capt to find assignments
  calculateDuration();    // Determine temporal scaling based on max dist

  mav_manager::GoalTimed srv;
  for(int i=0; i<num_active_bots_; i++){

    ROS_DEBUG_STREAM("Robot: " << i << " has goal number " << assignment_matrix_[i]);
    srv.request.goal[0] = goals_[assignment_matrix_[i]](0);
    srv.request.goal[1] = goals_[assignment_matrix_[i]](1);
    srv.request.goal[2] = goals_[assignment_matrix_[i]](2);
    srv.request.goal[3] = 0;  // TODO how to deal with individual robot yaw
    srv.request.duration = ros::Duration(duration_);
    srv.request.t_start = t_start_;

    // ROS_INFO_STREAM("service goes to " << srv.request.goal[0] << " with time " << srv.request.duration);
    if (!active_robots_[i]->sc_goToTimed_.call(srv)){
      res.message += active_robots_[i]->model_name_ + " failed during goForm";
      res.success = false;
    }
  }

  return true;
}

bool MMControl::checkCapt(multi_mav_manager::Formation::Response &res){

  std::vector<Eigen::Vector3f> positions = activeRobotPositions();

  for(int i=0; i<num_active_bots_; i++){
    for(int j=0; j<num_active_bots_; j++)
    {
      if(i != j){
        if((positions[i] - positions[j]).norm() < capt_spacing_)
        {
          float current_spacing = (positions[i] - positions[j]).norm();
          res.message += "\nRobots " + std::to_string(i) + " and " + std::to_string(j)
            + " are too close for capt. Current spacing: " + std::to_string(current_spacing) + " expected: " + std::to_string(capt_spacing_);
          res.success = false;
          ROS_WARN("Robots %d and %d are too close for capt. Current spacing: %f expected: %f", i, j, current_spacing, capt_spacing_);
        }
        if((goals_[i] - goals_[j]).norm() < capt_spacing_)
        {
          float current_spacing = (goals_[i] - goals_[j]).norm();
          res.message += "\nGoals " + std::to_string(i) + " and " + std::to_string(j)
            + " are too close for capt. Current spacing: " + std::to_string(current_spacing) + " expected: " + std::to_string(capt_spacing_);
          res.success = false;
          ROS_WARN("Goals %d and %d are too close for capt. Current spacing: %f expected: %f", i, j, (goals_[i] - goals_[j]).norm(), capt_spacing_);
        }
      }
    }
  }
  return res.success;
}

void MMControl::calculateDuration(){

  max_dist_ = 0.0;
  double dist_i = 0.0;

  auto positions = activeRobotPositions();

  for(int rob_i=0; rob_i<num_active_bots_; rob_i++){

    dist_i = (positions[rob_i] - goals_[assignment_matrix_[rob_i]]).norm();

    if(dist_i > max_dist_)  max_dist_ = dist_i;
  }

  // duration_ = std::sqrt(max_dist_) * 3; // TODO make a better heuristic for temporal scaling
  // duration_ = 0.2 + (max_dist_ / 0.25); // TODO Change to max velocity and min duration variables or params

  ROS_INFO("Using max_vel: %2.2f m/s and max_acc: %2.2f m/s^2 over a distance: %2.2f m", vmax_, amax_, max_dist_);

  // To determine the duration, see JT's matlab script in std_trackers/matlab
  duration_ = std::max(
      15.0 * max_dist_ / (8.0 * vmax_),
      std::sqrt((40.0 * std::sqrt(3.0) * max_dist_) / (3.0 * amax_)) / 2.0);

  t_start_ = ros::Time::now() + ros::Duration(num_active_bots_ * 0.02);  // Linearly scale start time by number of robots present
}

bool MMControl::calculateGoals(){

  // Define Rotation Matrices based on a yaw-pitch-roll euler angle rotation
  // Order is ZYX, yaw*pitch*roll
  const auto R = Eigen::AngleAxisf(formation_yaw_, Eigen::Vector3f::UnitZ()) *
                 Eigen::AngleAxisf(formation_pitch_, Eigen::Vector3f::UnitY()) *
                 Eigen::AngleAxisf(formation_roll_, Eigen::Vector3f::UnitX());

  bool valid_goals = true;
  // JT: Make sure goals_ is the correct size
  goals_.resize(num_active_bots_);
  for(size_t goal_i=0; goal_i<goals_.size(); goal_i++) {

    formation_offsets_[goal_i] =  R * formation_offsets_[goal_i];

    for(int dim_i=0; dim_i<3; dim_i++) {
      goals_[goal_i](dim_i) =
        formation_center_[dim_i] + formation_offsets_[goal_i](dim_i);
    }

    if((goals_[goal_i](0) < min_safety_bounds_.x) || (goals_[goal_i](0) > max_safety_bounds_.x) ){
      ROS_WARN("Goal %zu x: %2.2f outside the bound (%2.2f  %2.2f)", goal_i, goals_[goal_i](0),  min_safety_bounds_.x, max_safety_bounds_.x);
      valid_goals = false;
    }

    if((goals_[goal_i](1) < min_safety_bounds_.y) || (goals_[goal_i](1) > max_safety_bounds_.y) ){
      ROS_WARN("Goal %zu y: %2.2f outside the bound (%2.2f  %2.2f)", goal_i, goals_[goal_i](1),  min_safety_bounds_.y, max_safety_bounds_.y);
      valid_goals = false;
    }

    if((goals_[goal_i](2) < min_safety_bounds_.z) || (goals_[goal_i](2) > max_safety_bounds_.z) ){
      ROS_WARN("Goal %zu z: %2.2f outside the bound (%2.2f  %2.2f)", goal_i, goals_[goal_i](2),  min_safety_bounds_.z, max_safety_bounds_.z);
      valid_goals = false;
    }
  }

  //TODO check if formation roll/pitch causes vehicle one above each other, prevent downwash
  return valid_goals;
}

void MMControl::createDistMatrix(){

  std::vector<Eigen::Vector3f> positions = activeRobotPositions();

  capt_cost_ = 0;
  size_t num_goals = goals_.size();

  distMatrixSquared_ = std::vector<double>(num_active_bots_ * num_goals);

  for(size_t robot_i = 0; robot_i < positions.size(); robot_i++){
    for(size_t goal_i = 0; goal_i < num_goals; goal_i++){

      int q = (goal_i*num_goals) + robot_i;
      distMatrixSquared_[q] =
        (positions[robot_i] - goals_[goal_i]).squaredNorm();
    }
  }
}

bool MMControl::setDesVelInWorldFrame_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res) {
  return loop<mav_manager::Vec4>(req, res, &MavManagerInterface::sc_setDesVelInWorldFrame_, "setDesVelInWorldFrame");
}
bool MMControl::hover_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  return loop<std_srvs::Trigger>(req, res, &MavManagerInterface::sc_hover_, "hover");
}
bool MMControl::ehover_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  return loop<std_srvs::Trigger>(req, res, &MavManagerInterface::sc_ehover_, "ehover");
}
bool MMControl::land_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  return loop<std_srvs::Trigger>(req, res, &MavManagerInterface::sc_land_, "land");
}
bool MMControl::eland_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  return loop<std_srvs::Trigger>(req, res, &MavManagerInterface::sc_eland_, "eland");
}
bool MMControl::estop_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  return loop<std_srvs::Trigger>(req, res, &MavManagerInterface::sc_estop_, "estop");
}

template <typename T>
bool MMControl::loop(const typename T::Request &req, typename T::Response &res,
                     ros::ServiceClient MavManagerInterface::*sc, const std::string str)
{
  checkActiveRobots();
  T srv;
  srv.request = req;

  res.success = true;

  for(unsigned int i = 0; i < active_robots_.size(); i++)
  {
    // For the weird construction below, see https://stackoverflow.com/q/670734
    // and https://stackoverflow.com/a/21603687
    ros::ServiceClient &service = (*active_robots_[i]).*sc;
    if (!service.call(srv))
    {
      res.success = false;
      res.message = res.message + "\n\t" + active_robots_[i]->model_name_ + " failed to call " + str + ".\n" +
        "on service " + service.getService() + "\n";
    }
    else
    {
      res.success = res.success && srv.response.success;
      res.message = res.message + "\n\tResponse from " + active_robots_[i]->model_name_ + ": " + srv.response.message;
    }
    ros::Duration(0.01).sleep();
  }
  res.message = res.message + "\n";
  return true;
}


MMControl::MMControl() : nh_("multi_mav_services")
{
  // Get robot size/spacing information
  nh_.param("robot_radius", rob_radius_, 0.4);
  capt_spacing_ = rob_radius_ * 2 * std::sqrt(2);   // Minimum spacing required by capt to function
  default_spacing_ = 1.25 * capt_spacing_;           // If formation spacing is unspecified or too small, make it a safe one
  ROS_INFO_STREAM("Robot radius: " << rob_radius_ << " CAPT spacing: " << capt_spacing_);

  // Get information about usable robots
  nh_.getParam("model_names", model_names_);
  num_total_bots_ = model_names_.size();
  assignment_matrix_ = std::vector<int>(num_total_bots_);
  ROS_INFO_STREAM("Constructing multi_mav_control with " << num_total_bots_ << " indexed bots");

  nh_.param("max_vel", vmax_, 1.0);
  nh_.param("max_acc", amax_, 0.5);
  //nh_.param("minimum_height", minimum_height_, 0.1);
  ROS_INFO_STREAM("Max vel: " << vmax_ << " Max acc: " << amax_);

  std::string odom_topic, goto_base_name;
  nh_.param("odom_topic", odom_topic, std::string("odom_static"));
  nh_.param("goto_base_name", goto_base_name, std::string("mav_services"));

  std::vector<double> min_bounds, max_bounds;
  nh_.getParam("min_safety_bounds", min_bounds);
  if(min_bounds.size() != 3)
  {
    ROS_WARN("Check safety bounds, size should be 3, setting default");
    min_safety_bounds_.x = -4.0; min_safety_bounds_.y = -4.0; min_safety_bounds_.z = 0.2;
  }
  else
  {
    min_safety_bounds_.x = min_bounds[0]; min_safety_bounds_.y = min_bounds[1]; min_safety_bounds_.z = min_bounds[2];
  }
  nh_.getParam("max_safety_bounds", max_bounds);
  if(max_bounds.size() != 3)
  {
    ROS_WARN("Check safety bounds, size should be 3, setting default");
    max_safety_bounds_.x = 4.0; max_safety_bounds_.y = 4.0; max_safety_bounds_.z = 4.0;
  }
  else
  {
    max_safety_bounds_.x = max_bounds[0]; max_safety_bounds_.y = max_bounds[1]; max_safety_bounds_.z = max_bounds[2];
  }

  ROS_INFO("Safety bounds min: (%2.2f, %2.2f, %2.2f) max: (%2.2f, %2.2f, %2.2f)",
      min_safety_bounds_.x, min_safety_bounds_.y, min_safety_bounds_.z,
      max_safety_bounds_.x, max_safety_bounds_.y, max_safety_bounds_.z);

  bool act = false;
  float battery_low;
  nh_.param<float>("battery_low", battery_low, 4.0);

  for(int i = 0; i < num_total_bots_; i++)
  {
    nh_.getParam("/" + model_names_[i] + "/active", act);
    auto mmi = std::make_shared<MavManagerInterface>(model_names_[i], odom_topic, goto_base_name, act, battery_low, this);
    robots_.push_back(mmi);
    //if(act)   active_robots_.push_back(mmi);  // Add to active robots list
  }

  num_active_bots_ = robots_.size();
  formation_offsets_.resize(num_total_bots_);

  srv_motors_                 = nh_.advertiseService("motors", &MMControl::motors_cb, this);
  srv_takeoff_                = nh_.advertiseService("takeoff", &MMControl::takeoff_cb, this);
  // srv_goHome_                 = nh_.advertiseService("goHome", &MMControl::goHome_cb, this);
  srv_goTo_                   = nh_.advertiseService("goTo", &MMControl::goTo_cb, this);
  srv_goToTimed_              = nh_.advertiseService("goToTimed", &MMControl::goToTimed_cb, this);
  srv_setDesVelInWorldFrame_  = nh_.advertiseService("setDesVelInWorldFrame", &MMControl::setDesVelInWorldFrame_cb, this);
  srv_hover_                  = nh_.advertiseService("hover", &MMControl::hover_cb, this);
  srv_ehover_                 = nh_.advertiseService("ehover", &MMControl::ehover_cb, this);
  srv_land_                   = nh_.advertiseService("land", &MMControl::land_cb, this);
  srv_eland_                  = nh_.advertiseService("eland", &MMControl::eland_cb, this);
  srv_estop_                  = nh_.advertiseService("estop", &MMControl::estop_cb, this);
  srv_goFormRawPos_           = nh_.advertiseService("goFormRawPos", &MMControl::goFormRawPos_cb, this);
  srv_goFormCircle_           = nh_.advertiseService("goFormCircle", &MMControl::goFormCircle_cb, this);
  srv_goFormLine_             = nh_.advertiseService("goFormLine", &MMControl::goFormLine_cb, this);
  srv_goFormRect_             = nh_.advertiseService("goFormRect", &MMControl::goFormRect_cb, this);
  srv_goFormGrid3d_           = nh_.advertiseService("goFormGrid3d", &MMControl::goFormGrid3d_cb, this);
  srv_goFormAngle_            = nh_.advertiseService("goFormAngle", &MMControl::goFormAngle_cb, this);
  srv_goFormTriangle_         = nh_.advertiseService("goFormTriangle", &MMControl::goFormTriangle_cb, this);

  ros::Duration(1.0).sleep(); //Why?
  checkActiveRobots();
  ROS_WARN_STREAM("================== Multi Mav Manager is ready for action ==================");
}

// Function gets called when a robot changes it's active status
void MMControl::checkActiveRobots()
{
  active_robots_.clear();   // Remove all robots from active list

  // TODO: monitor for changes in active states, do stuff if that changes

  std::cout << "Active robots are: " << std::endl;
  for(unsigned int i = 0; i < robots_.size(); i++){
    if(robots_[i]->isActive()){
      active_robots_.push_back(robots_[i]);
      std::cout << "\t" << robots_[i]->model_name_ << std::endl;
    }
  }
  num_active_bots_ = active_robots_.size();

  // This is supposed to have the robots automatically reform their formation any time a robot changes its active status
  // but it is much harder than I wanted it to be -Aaron
}

std::vector<Eigen::Vector3f> MMControl::activeRobotPositions()
{
  std::vector<Eigen::Vector3f> positions;
  positions.resize(active_robots_.size());

  for(unsigned int i = 0; i < active_robots_.size(); i++)
    positions[i] = active_robots_[i]->getPosition();

  return positions;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_mav_services");

  MMControl multi_mav_control;

  ros::spin();

  return 0;
}
