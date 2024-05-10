#include <leader_follower_task/message_utils.h>
#include <nav_msgs/Odometry.h>

mrs_msgs::ReferenceStamped buildMpcReference(Eigen::Vector3d position, double heading, std::string frame) {
  mrs_msgs::ReferenceStamped ref;
  ref.header.stamp         = ros::Time::now();
  ref.header.frame_id      = frame;
  ref.reference.heading    = heading;
  ref.reference.position.x = position.x();
  ref.reference.position.y = position.y();
  ref.reference.position.z = position.z();
  return ref;
}


mrs_msgs::ReferenceStamped buildMpcReference(Eigen::Vector3d desired_position, std::string frame) {
  mrs_msgs::ReferenceStamped ref;
  ref.header.stamp         = ros::Time::now();
  ref.header.frame_id      = frame;
  ref.reference.position.x = desired_position.x();
  ref.reference.position.y = desired_position.y();
  ref.reference.position.z = desired_position.z();
  return ref;
}

mrs_msgs::TrajectoryReference buildMpcTrajectoryReference(std::vector<Eigen::Vector3d> positions, std::vector<double> headings, double sampling_time,
                                                          std::string frame) {
  // sanity checks
  if (positions.size() < 1 || headings.size() < 1) {
    ROS_ERROR("[%s]: Cannot generate empty trajectory!", ros::this_node::getName().c_str());
  }
  if (positions.size() != headings.size()) {
    ROS_WARN("[%s]: Data length mismatch! Tried to build a trajectory with %ld points, but %ld headings. Discarding extra values.",
             ros::this_node::getName().c_str(), positions.size(), headings.size());
  }
  size_t data_size = std::min(positions.size(), headings.size());

  mrs_msgs::TrajectoryReference traj;
  traj.header.stamp    = ros::Time::now();
  traj.header.frame_id = frame;
  traj.dt              = sampling_time;
  traj.fly_now         = true;
  traj.loop            = false;
  traj.use_heading     = true;

  for (size_t i = 0; i < data_size; i++) {
    mrs_msgs::Reference r;
    r.position.x = positions[i].x();
    r.position.y = positions[i].y();
    r.position.z = positions[i].z();
    r.heading    = headings[i];
    traj.points.push_back(r);
  }
  return traj;
}

mrs_msgs::TrajectoryReference buildMpcTrajectoryReference(std::vector<Eigen::Vector3d> position, double sampling_time, std::string frame) {
  // sanity check
  if (position.size() < 1) {
    ROS_ERROR("[%s]: Cannot generate empty trajectory!", ros::this_node::getName().c_str());
  }

  mrs_msgs::TrajectoryReference traj;
  traj.header.stamp    = ros::Time::now();
  traj.header.frame_id = frame;
  traj.dt              = sampling_time;
  traj.fly_now         = true;
  traj.loop            = false;
  traj.use_heading     = false;

  for (size_t i = 0; i < position.size(); i++) {
    mrs_msgs::Reference r;
    r.position.x = position[i].x();
    r.position.y = position[i].y();
    r.position.z = position[i].z();
    traj.points.push_back(r);
  }
  return traj;
}

mrs_msgs::SpeedTrackerCommand buildSpeedTrackerCommand(Eigen::Vector3d velocity, double heading, double height, std::string frame) {
  mrs_msgs::SpeedTrackerCommand cmd;
  cmd.header.frame_id = frame;
  cmd.header.stamp    = ros::Time::now();
  cmd.velocity.x      = velocity.x();
  cmd.velocity.y      = velocity.y();
  cmd.velocity.z      = 0.0;   // let the height controller maintain the vertical velocity
  cmd.use_velocity    = true;  // maintain the desired velocity
  cmd.z               = height;
  cmd.use_z           = true;  // maintain a constant height
  cmd.heading         = heading;
  cmd.use_heading     = true;
  return cmd;
}

nav_msgs::Odometry buildOdometryMessage(Eigen::Vector3d position, Eigen::Vector3d velocity, std::string frame) {
  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id      = frame;
  odom_msg.header.stamp         = ros::Time::now();
  odom_msg.pose.pose.position.x = position.x();
  odom_msg.pose.pose.position.y = position.y();
  odom_msg.pose.pose.position.z = position.z();
  odom_msg.twist.twist.linear.x = velocity.x();
  odom_msg.twist.twist.linear.y = velocity.y();
  odom_msg.twist.twist.linear.z = velocity.z();
  return odom_msg;
}
