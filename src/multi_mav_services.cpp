#include <multi_mav_manager/multi_mav_services.h>

#include <hungarian.h>

bool MMControl::motors_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
  return loop<std_srvs::SetBool>(req, res, &MavManagerInterface::sc_motors, "motors");
}
bool MMControl::takeoff_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  return loop<std_srvs::Trigger>(req, res, &MavManagerInterface::sc_takeoff, "takeoff");
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
  if (req.goals.size() != (unsigned)num_active_bots)
  {
    // TODO handle case where goals are different length than robots better
    res.success = false;
    res.message = "req.goals.size() != num_active_bots";
    return true;
  }

  // TODO: change this sizing thing
  for(int i = 0; i < num_active_bots; i++)
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
  formation_shape_ = line;
  cleanFormation(req, res);

  for(int i=0; i<num_active_bots; i++){
    formation_offsets_[i][0] = (i-((num_active_bots-1)/2.0)) * req.spacing;
    formation_offsets_[i][1] = 0.0;
    formation_offsets_[i][2] = 0.0;
  }

  return goForm(req, res);
}

bool MMControl::goFormTriangle_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res)
{
  formation_shape_ = triangle;
  cleanFormation(req, res);

  double R = req.spacing;
  double th = M_PI / 3.0;

  if(num_active_bots == 6){
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
  formation_shape_ = angle;
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

  for(int i=0; i<num_active_bots; i++){
    formation_offsets_[i][0] = -1 * radius * std::cos(angle/2);
    formation_offsets_[i][1] = radius * x_side * std::sin(angle/2);
    formation_offsets_[i][2] = 0;

    x_side = x_side * -1;
    if(x_side == -1)   radius += req.spacing;
  }

  return goForm(req, res);
}

bool MMControl::goFormCircle_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res){

  formation_shape_ = circle;
  cleanFormation(req, res);

  double radius = (num_active_bots * req.spacing) / (2 * M_PI);

  for(int i = 0; i < num_active_bots; i++)
  {
    float theta_i = i * (2 * M_PI / num_active_bots);

    formation_offsets_[i](0) = radius * std::cos(theta_i);
    formation_offsets_[i](1) = radius * std::sin(theta_i);
    formation_offsets_[i](2) = 0;
  }

  return goForm(req, res);
}

bool MMControl::goFormRect_cb(multi_mav_manager::Formation::Request &req, multi_mav_manager::Formation::Response &res){

  formation_shape_ = rect;
  cleanFormation(req, res);

  int min_square_root = floor(sqrt(num_active_bots));
  int rows = min_square_root;
  int cols = 1;

  for(int rows = min_square_root; rows >=1; rows-=1){
    if(num_active_bots % rows == 0){
      cols = num_active_bots / rows;
      break;
    }
  }

  for(int i = 0; i < num_active_bots; i++)
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

  formation_shape_ = grid3d;
  cleanFormation(req, res);

  int min_cube_root = floor(std::pow(num_active_bots, 1.0/3.0));

  int lays = min_cube_root; // Layers
  int rows = 1;
  int cols = 1;
  bool grid_found = false;

  while(!grid_found && lays >= 1)
  {
    if(num_active_bots % lays == 0){

      rows = floor(std::pow((num_active_bots / lays), 1.0/2.0));

      while(!grid_found && rows >= 1)
      {
        ROS_INFO_STREAM("trying rows = " << rows);
        if((num_active_bots / lays) % rows == 0){

          ROS_INFO_STREAM("found rows = " << rows);
          cols = (num_active_bots/lays) / rows;

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

  for(int i = 0; i < num_active_bots; i++)
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
    res.message += "\nRequested spacing is too small, set to default: " + std::to_string(default_spacing);
    req.spacing = default_spacing;
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

  formation_roll_ = req.roll;
  formation_pitch_ = req.pitch;
  formation_yaw_ = req.yaw;

  calculateGoals();       // Calculate global goal locations from formation information

  ROS_INFO("Successfully calcuated goals.");
  if(!checkCapt(res)){
    ROS_INFO("Issue here? Check that initial and goal locations are properly spaced");
    return true;            // Check that initial and goal locations are properly spaced
  }

  ROS_INFO("CAPT...");
  createDistMatrix();     // Calculated the distance matrix used by capt
  Hungarian h(assignment_matrix_.data(), &capt_cost_, distMatrixSquared_.data(),
              num_active_bots, goals_.size()); // Create hungarian object for capt
  h.computeAssignment();  // Use capt to find assignments
  calculateDuration();    // Determine temporal scaling based on max dist

  mav_manager::GoalTimed srv;

  for(int i=0; i<num_active_bots; i++){

    ROS_DEBUG_STREAM("Robot: " << i << " has goal number " << assignment_matrix_[i]);
    srv.request.goal[0] = goals_[assignment_matrix_[i]](0);
    srv.request.goal[1] = goals_[assignment_matrix_[i]](1);
    srv.request.goal[2] = goals_[assignment_matrix_[i]](2);
    srv.request.goal[3] = 0;  // TODO how to deal with individual robot yaw
    srv.request.duration = ros::Duration(duration_);
    srv.request.t_start = t_start_;

    // ROS_INFO_STREAM("service goes to " << srv.request.goal[0] << " with time " << srv.request.duration);

    if (!active_robots_[i]->sc_goToTimed.call(srv))
    {
      res.message += active_robots_[i]->model_name_ + " failed during goForm";
      res.success = false;
    }
  }

  return true;
}

bool MMControl::checkCapt(multi_mav_manager::Formation::Response &res){

  std::vector<Eigen::Vector3f> positions = activeRobotPositions();

  ROS_INFO("positions.size(): %lu", positions.size());
  for(int i=0; i<num_active_bots; i++){
    for(int j=0; j<num_active_bots; j++)
    {
      if(i != j){
        if((positions[i] - positions[j]).norm() < capt_spacing_)
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

void MMControl::calculateDuration(){

  max_dist_ = 0.0;
  double dist_i = 0.0;

  auto positions = activeRobotPositions();

  for(int rob_i=0; rob_i<num_active_bots; rob_i++){

    dist_i = (positions[rob_i] - goals_[assignment_matrix_[rob_i]]).norm();

    if(dist_i > max_dist_)  max_dist_ = dist_i;
  }

  // duration_ = std::sqrt(max_dist_) * 3; // TODO make a better heuristic for temporal scaling
  // duration_ = 0.2 + (max_dist_ / 0.25); // TODO Change to max velocity and min duration variables or params

  float vmax(2.0), amax(1.0);
  nh.getParam("max_vel", vmax);
  nh.getParam("max_acc", amax);
  ROS_INFO("Using max_vel = %2.2f m/s and max_acc = %2.2f m/s^2 over a distance of %2.2f m", vmax, amax, max_dist_);

  // To determine the duration, see JT's matlab script in std_trackers/matlab
  duration_ = std::max(
      15.0 * max_dist_ / (8.0 * vmax),
      std::sqrt((40.0 * std::sqrt(3.0) * max_dist_) / (3.0 * amax)) / 2.0);

  t_start_ = ros::Time::now() + ros::Duration(num_active_bots * 0.02);  // Linearly scale start time by number of robots present
}

void MMControl::calculateGoals(){

  // Define Rotation Matrices based on a yaw-pitch-roll euler angle rotation
  // Order is ZYX, yaw*pitch*roll
  const auto R = Eigen::AngleAxisf(formation_yaw_, Eigen::Vector3f::UnitZ()) *
                 Eigen::AngleAxisf(formation_pitch_, Eigen::Vector3f::UnitY()) *
                 Eigen::AngleAxisf(formation_roll_, Eigen::Vector3f::UnitX());

  // JT: Make sure goals_ is the correct size
  goals_.resize(num_active_bots);
  for(size_t goal_i=0; goal_i<goals_.size(); goal_i++) {

    formation_offsets_[goal_i] =  R * formation_offsets_[goal_i];

    for(int dim_i=0; dim_i<3; dim_i++) {
      goals_[goal_i](dim_i) =
        formation_center_[dim_i] + formation_offsets_[goal_i](dim_i);
    }

    if(goals_[goal_i](2) < 0.0f) {
      ROS_INFO("Goal %zu was too low at %2.2f so I set it to 0", goal_i, goals_[goal_i](2));
      goals_[goal_i](2) = 0.0f;   // Protect against robots crashing into the ground
      // TODO make minimum height a param
      // TODO make the boundaries of the space a param
    }
  }
}

void MMControl::createDistMatrix(){

  std::vector<Eigen::Vector3f> positions = activeRobotPositions();

  capt_cost_ = 0;
  size_t num_goals = goals_.size();

  distMatrixSquared_ = std::vector<double>(num_active_bots * num_goals);

  for(size_t robot_i = 0; robot_i < positions.size(); robot_i++){
    for(size_t goal_i = 0; goal_i < num_goals; goal_i++){

      int q = (goal_i*num_goals) + robot_i;
      distMatrixSquared_[q] =
        (positions[robot_i] - goals_[goal_i]).squaredNorm();
    }
  }
}

bool MMControl::setDesVelInWorldFrame_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res) {
  return loop<mav_manager::Vec4>(req, res, &MavManagerInterface::sc_setDesVelInWorldFrame, "setDesVelInWorldFrame");
}
bool MMControl::hover_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  return loop<std_srvs::Trigger>(req, res, &MavManagerInterface::sc_hover, "hover");
}
bool MMControl::ehover_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  return loop<std_srvs::Trigger>(req, res, &MavManagerInterface::sc_ehover, "ehover");
}
bool MMControl::land_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  return loop<std_srvs::Trigger>(req, res, &MavManagerInterface::sc_land, "land");
}
bool MMControl::eland_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  return loop<std_srvs::Trigger>(req, res, &MavManagerInterface::sc_eland, "eland");
}
bool MMControl::estop_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  return loop<std_srvs::Trigger>(req, res, &MavManagerInterface::sc_estop, "estop");
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


MMControl::MMControl() : nh("multi_mav_services"), priv_nh("")
{
  ros::Duration(5.0).sleep();

  // Get robot size/spacing information
  nh.getParam("robot_radius", rob_radius_);
  std::cout << "Robot radius is set to " << rob_radius_ << std::endl;
  capt_spacing_ = rob_radius_ * 2 * std::sqrt(2);   // Minimum spacing required by capt to function
  default_spacing = 1.25 * capt_spacing_;           // If formation spacing is unspecified or too small, make it a safe one

  // Get information about usable robots
  nh.getParam("model_names", model_names_);
  num_total_bots = model_names_.size();

  std::cout << "Constructing multi_mav_control with " << num_total_bots << " indexed bots" << std::endl;

  assignment_matrix_ = std::vector<int>(num_total_bots);
  bool act = false;

  for(int i = 0; i < num_total_bots; i++)
  {
    nh.getParam("/" + model_names_[i] + "/active", act);

    float battery_low = 4;
    nh.getParam("battery_low", battery_low);

    auto mmi = std::make_shared<MavManagerInterface>(model_names_[i], act, battery_low, this);

    robots_.push_back(mmi);
    //if(act)   active_robots_.push_back(mmi);  // Add to active robots list
  }

  num_active_bots = robots_.size();
  formation_offsets_.resize(num_total_bots);

  srv_motors_                 = nh.advertiseService("motors", &MMControl::motors_cb, this);
  srv_takeoff_                = nh.advertiseService("takeoff", &MMControl::takeoff_cb, this);
  // srv_goHome_                 = nh.advertiseService("goHome", &MMControl::goHome_cb, this);
  srv_goTo_                   = nh.advertiseService("goTo", &MMControl::goTo_cb, this);
  srv_goToTimed               = nh.advertiseService("goToTimed", &MMControl::goToTimed_cb, this);
  srv_setDesVelInWorldFrame_  = nh.advertiseService("setDesVelInWorldFrame", &MMControl::setDesVelInWorldFrame_cb, this);
  srv_hover_                  = nh.advertiseService("hover", &MMControl::hover_cb, this);
  srv_ehover_                 = nh.advertiseService("ehover", &MMControl::ehover_cb, this);
  srv_land_                   = nh.advertiseService("land", &MMControl::land_cb, this);
  srv_eland_                  = nh.advertiseService("eland", &MMControl::eland_cb, this);
  srv_estop_                  = nh.advertiseService("estop", &MMControl::estop_cb, this);
  srv_goFormRawPos_           = nh.advertiseService("goFormRawPos", &MMControl::goFormRawPos_cb, this);
  srv_goFormCircle_           = nh.advertiseService("goFormCircle", &MMControl::goFormCircle_cb, this);
  srv_goFormLine_             = nh.advertiseService("goFormLine", &MMControl::goFormLine_cb, this);
  srv_goFormRect_             = nh.advertiseService("goFormRect", &MMControl::goFormRect_cb, this);
  srv_goFormGrid3d_           = nh.advertiseService("goFormGrid3d", &MMControl::goFormGrid3d_cb, this);
  srv_goFormAngle_            = nh.advertiseService("goFormAngle", &MMControl::goFormAngle_cb, this);
  srv_goFormTriangle_         = nh.advertiseService("goFormTriangle", &MMControl::goFormTriangle_cb, this);

  std::cout << "================== Multi Mav Manager is ready for action ==================" << std::endl;
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
  num_active_bots = active_robots_.size();

  // This is supposed to have the robots automatically reform their formation any time a robot changes its active status
  // but it is much harder than I wanted it to be -Aaron
}

std::vector<Eigen::Vector3f> MMControl::activeRobotPositions()
{
  std::vector<Eigen::Vector3f> positions;
  positions.resize(active_robots_.size());

  for(unsigned int i = 0; i < active_robots_.size(); i++)
    positions[i] = active_robots_[i]->position_;

  return positions;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_mav_services");

  MMControl multi_mav_control;

  ros::spin();

  return 0;
}
