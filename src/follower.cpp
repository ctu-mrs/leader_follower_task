#include <leader_follower_task/follower.h>
#include <leader_follower_task/velocity_estimator.h>
#include <leader_follower_task/FollowerConfig.h>

bool is_initialized     = false;
bool got_odometry       = false;
bool got_tracker_output = false;
bool got_uvdar          = false;

Eigen::Vector3d follower_position_odometry;
Eigen::Vector3d follower_linear_velocity_odometry;
double          follower_heading_odometry;
double          follower_heading_rate_odometry;

Eigen::Vector3d follower_position_tracker;
Eigen::Vector3d follower_linear_velocity_tracker;
double          follower_heading_tracker;
double          follower_heading_rate_tracker;

Eigen::Vector3d leader_position;
ros::Time       last_leader_contact;

// dynamically reconfigurable
Eigen::Vector3d position_offset          = Eigen::Vector3d(0.0, 0.0, 0.0);
double          heading_offset           = 0.0;
double          uvdar_msg_interval       = 0.1;
bool            use_estimator            = false;
bool            use_speed_tracker        = false;
bool            use_trajectory_reference = false;

VelocityEstimator estimator;
Eigen::Vector3d   leader_predicted_position;
Eigen::Vector3d   leader_predicted_velocity;

/* initialize //{ */
leader_follower_task::FollowerConfig FollowerController::initialize(mrs_lib::ParamLoader& param_loader) {

  ROS_INFO("[Follower]: Waiting for odometry and uvdar");

  while (ros::ok()) {

    if (got_odometry && got_uvdar) {
      break;
    } else {
      ROS_INFO_THROTTLE(1.0, "[Follower]: waiting, got ODOMETRY=%s, UVDAR=%s", got_odometry ? "TRUE" : "FALSE", got_uvdar ? "TRUE" : "FALSE");
    }

    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  Eigen::Matrix<double, 6, 6> Q;
  Eigen::Matrix<double, 3, 3> R;
  param_loader.loadMatrixStatic("Q", Q);
  param_loader.loadMatrixStatic("R", R);
  param_loader.loadParam("control_action_interval", control_action_interval);
  param_loader.loadParam("desired_offset/x", position_offset.x());
  param_loader.loadParam("desired_offset/y", position_offset.y());
  param_loader.loadParam("desired_offset/z", position_offset.z());
  param_loader.loadParam("heading_offset", heading_offset);

  //// initialize the dynamic reconfigurables with values from YAML file and values set above
  leader_follower_task::FollowerConfig config;
  config.desired_offset_x         = position_offset.x();
  config.desired_offset_y         = position_offset.y();
  config.desired_offset_z         = position_offset.z();
  config.heading_offset           = heading_offset;
  config.filter_data              = use_estimator;
  config.use_trajectory_reference = use_trajectory_reference;
  config.use_speed_tracker        = use_speed_tracker;
  ////

  VelocityEstimator::kalman3D::x_t initial_states;

  // set initial state of estimator as follows: leader position: (current follower pos - desired offset), leader velocity: (0,0,0)
  initial_states << follower_position_odometry.x() - position_offset.x(), follower_position_odometry.y() - position_offset.y(),
      follower_position_odometry.z() - position_offset.z(), 0, 0, 0;
  estimator = VelocityEstimator(Q, R, initial_states, uvdar_msg_interval);


  is_initialized = true;
  return config;
}
//}

/* dynamicReconfigureCallback //{ */
void FollowerController::dynamicReconfigureCallback(leader_follower_task::FollowerConfig& config, [[maybe_unused]] uint32_t level) {
  position_offset          = Eigen::Vector3d(config.desired_offset_x, config.desired_offset_y, config.desired_offset_z);
  heading_offset           = config.heading_offset;
  use_speed_tracker        = config.use_speed_tracker;
  use_trajectory_reference = config.use_trajectory_reference;

  if (!use_estimator && config.filter_data) {
    ROS_INFO("[%s]: Estimator started", ros::this_node::getName().c_str());
  }
  use_estimator = config.filter_data;
}
//}

/* receiveOdometry //{ */
void FollowerController::receiveOdometry(const nav_msgs::Odometry& odometry_msg) {

  follower_position_odometry.x() = odometry_msg.pose.pose.position.x;
  follower_position_odometry.y() = odometry_msg.pose.pose.position.y;
  follower_position_odometry.z() = odometry_msg.pose.pose.position.z;

  mrs_lib::AttitudeConverter ac(odometry_msg.pose.pose.orientation);
  follower_heading_odometry = ac.getHeading();

  follower_linear_velocity_odometry.x() = odometry_msg.twist.twist.linear.x;
  follower_linear_velocity_odometry.y() = odometry_msg.twist.twist.linear.y;
  follower_linear_velocity_odometry.z() = odometry_msg.twist.twist.linear.z;

  follower_heading_rate_odometry =
      ac.getHeadingRate(Eigen::Vector3d(odometry_msg.twist.twist.angular.x, odometry_msg.twist.twist.angular.y, odometry_msg.twist.twist.angular.z));

  got_odometry = true;
}
//}

/* receiveTrackerOutput //{ */
void FollowerController::receiveTrackerOutput(const mrs_msgs::TrackerCommand& position_cmd) {

  follower_position_tracker.x() = position_cmd.position.x;
  follower_position_tracker.y() = position_cmd.position.y;
  follower_position_tracker.z() = position_cmd.position.z;

  follower_heading_tracker = position_cmd.heading;

  follower_linear_velocity_tracker.x() = position_cmd.velocity.x;
  follower_linear_velocity_tracker.y() = position_cmd.velocity.y;
  follower_linear_velocity_tracker.z() = position_cmd.velocity.z;

  follower_heading_rate_tracker = position_cmd.heading_rate;

  got_tracker_output = true;
}
//}

/* receiveUvdar //{ */
void FollowerController::receiveUvdar(const geometry_msgs::PoseWithCovarianceStamped& uvdar_msg) {

  Eigen::Vector3d leader_new_position;

  leader_new_position.x() = uvdar_msg.pose.pose.position.x;
  leader_new_position.y() = uvdar_msg.pose.pose.position.y;
  leader_new_position.z() = uvdar_msg.pose.pose.position.z;

  last_leader_contact = uvdar_msg.header.stamp;
  got_uvdar           = true;

  leader_position = leader_new_position;

  if (use_estimator && is_initialized) {
    estimator.fuse(leader_new_position);
  }
}
//}

/* createReferencePoint //{ */
ReferencePoint FollowerController::createReferencePoint() {
  ReferencePoint point;

  // sanity check
  if (!is_initialized || !got_odometry || !got_uvdar || !got_tracker_output) {
    point.position        = Eigen::Vector3d(0, 0, 0);
    point.heading         = 0;
    point.use_for_control = false;
    return point;
  }

  if (use_estimator) {
    point.position.x() = leader_predicted_position.x() + position_offset.x();
    point.position.y() = leader_predicted_position.y() + position_offset.y();
    point.position.z() = 3.0;
  } else {
    point.position.x() = leader_position.x() + position_offset.x();
    point.position.y() = leader_position.y() + position_offset.y();
    point.position.z() = 3.0;
  }
  point.heading         = heading_offset;
  point.use_for_control = true;

  return point;
}
//}

/* createReferenceTrajectory //{ */
ReferenceTrajectory FollowerController::createReferenceTrajectory() {
  ReferenceTrajectory trajectory;

  // sanity check
  if (!is_initialized || !got_odometry || !got_uvdar || !got_tracker_output) {
    trajectory.positions.push_back(Eigen::Vector3d::Zero());
    trajectory.headings.push_back(0.0);
    trajectory.sampling_time   = 0.0;
    trajectory.use_for_control = false;
    return trajectory;
  }

  // Example - start trajectory at current UAV position and move in the predicted direction of leader motion
  // No subsampling, only two points are created in this example
  Eigen::Vector3d point_1;
  double          heading_1;

  Eigen::Vector3d point_2;
  double          heading_2;

  trajectory.use_for_control = false;
  if (use_trajectory_reference) {
    if (use_estimator) {
      point_1   = follower_position_tracker;
      heading_1 = follower_heading_tracker;

      point_2   = leader_predicted_position + position_offset + (leader_predicted_velocity * control_action_interval);
      heading_2 = heading_offset;

      trajectory.positions.push_back(point_1);
      trajectory.positions.push_back(point_2);

      trajectory.headings.push_back(heading_1);
      trajectory.headings.push_back(heading_2);
      trajectory.sampling_time   = control_action_interval;
      trajectory.use_for_control = true;
    } else {
      ROS_WARN("[%s]: Tried to plan a trajectory without leader velocity estimation", ros::this_node::getName().c_str());
    }
  }

  return trajectory;
}
//}

/* createSpeedCommand //{ */
SpeedCommand FollowerController::createSpeedCommand() {
  SpeedCommand command;

  if (!got_odometry || !got_uvdar || !got_tracker_output) {
    command.velocity        = Eigen::Vector3d(0, 0, 0);
    command.heading         = 0;
    command.height          = 0;
    command.use_for_control = false;
  }

  if (use_estimator) {
    command.velocity = leader_predicted_velocity;
    command.height   = leader_predicted_position.z() + position_offset.z();
    command.heading  = follower_heading_odometry;
  }

  if (use_speed_tracker) {
    command.use_for_control = true;
  } else {
    command.use_for_control = false;
  }
  return command;
}
//}

/* getCurrentEstimate //{ */

// You can use this method for debugging purposes.
// It allows you to visualize the leader predictions in rviz
// It is called once per control action of the summer_schoo_supervisor

nav_msgs::Odometry FollowerController::getCurrentEstimate() {
  nav_msgs::Odometry leader_est;

  if (use_estimator) {
    auto leader_prediction          = estimator.predict(Eigen::Vector3d(0, 0, 0), control_action_interval);
    leader_predicted_position       = Eigen::Vector3d(leader_prediction[0], leader_prediction[1], leader_prediction[2]);
    leader_predicted_velocity       = Eigen::Vector3d(leader_prediction[3], leader_prediction[4], leader_prediction[5]);
    leader_est.pose.pose.position.x = leader_prediction[0];
    leader_est.pose.pose.position.y = leader_prediction[1];
    leader_est.pose.pose.position.z = leader_prediction[2];
    leader_est.twist.twist.linear.x = leader_prediction[3];
    leader_est.twist.twist.linear.y = leader_prediction[4];
    leader_est.twist.twist.linear.z = leader_prediction[5];
  } else {
    leader_est.pose.pose.position.x = leader_position.x();
    leader_est.pose.pose.position.y = leader_position.y();
    leader_est.pose.pose.position.z = leader_position.z();
  }

  return leader_est;
}
//}
