#include <leader_follower_task/follower.h>
#include <leader_follower_task/velocity_estimator.h>
#include <leader_follower_task/FollowerConfig.h>
#include <cmath> 

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
double leader_heading;
ros::Time       last_leader_contact;

// dynamically reconfigurable
Eigen::Vector3d position_offset          = Eigen::Vector3d(0.0, 0.0, 0.0);
double          heading_offset           = 0.0;
double          uvdar_msg_interval       = 0.1;

bool            use_estimator            = true;
bool            use_trajectory_reference = false;
bool            use_velocity_reference   = false;

VelocityEstimator estimator;
Eigen::Vector3d   leader_predicted_position;
Eigen::Vector3d   leader_predicted_velocity;

ReferenceTrajectory leader_trajectory;
double epsilon = 3;

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
  config.use_velocity_reference   = use_velocity_reference;
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
  use_trajectory_reference = config.use_trajectory_reference;
  use_velocity_reference   = config.use_velocity_reference;

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

  mrs_lib::AttitudeConverter ac(uvdar_msg.pose.pose.orientation);
  leader_heading = ac.getHeading();

  last_leader_contact = uvdar_msg.header.stamp;
  got_uvdar           = true;

  leader_position = leader_new_position;

  if (use_estimator && is_initialized) {
    estimator.fuse(leader_new_position);
  }
/*
  leader_trajectory.positions.push_back(leader_position);
  leader_trajectory.headings.push_back(leader_heading);
*/
/*
	ROS_INFO("point added: x=%f, y=%f, z=%f , heading=%f", leader_position.x(), leader_position.y(), leader_position.z(), leader_heading);
	ROS_INFO("leader_trajectory size - Positions:%lu -Headings:%lu", leader_trajectory.positions.size(), leader_trajectory.headings.size());
*/
}
//}

/* createReferencePoint //{ */
ReferencePoint FollowerController::createReferencePoint() {
  ReferencePoint point;

/*
  leader_trajectory.positions.push_back(leader_predicted_position);
  leader_trajectory.headings.push_back(leader_heading);
*/
  // sanity check
  if (!is_initialized || !got_odometry || !got_uvdar || !got_tracker_output) {
    point.position        = Eigen::Vector3d(0, 0, 0);
    point.heading         = 0;
    point.use_for_control = false;
    return point;
  }
/*
  ROS_INFO("Point :x=%f, y=%f, z=%f, heading=%f", leader_trajectory.positions[0].x(), leader_trajectory.positions[0].y(), leader_trajectory.positions[0].z(), leader_trajectory.headings[0]);
  ROS_INFO("Follower_pos :x=%f, y=%f, z=%f, heading=%f", follower_position_tracker.x(), follower_position_tracker.y(), follower_position_tracker.z(), follower_heading_tracker);

  ROS_INFO("Cond_1 =%f, Cond_2 =%f", std::abs(leader_trajectory.positions[0].x() - follower_position_tracker.x()), std::abs(leader_trajectory.positions[0].y() - follower_position_tracker.y()));

  if (!leader_trajectory.positions.empty() && !leader_trajectory.headings.empty() &&
    std::abs(leader_trajectory.positions[0].x() - follower_position_tracker.x()) < epsilon &&
    std::abs(leader_trajectory.positions[0].y() - follower_position_tracker.y()) < epsilon ) {

    ROS_INFO("Taille avant suppression - Positions: %lu, Heading: %lu",
    leader_trajectory.positions.size(), leader_trajectory.headings.size());

    ROS_INFO("Point_0 avant supression :x=%f, y=%f, z=%f, heading=%f", leader_trajectory.positions[0].x(), leader_trajectory.positions[0].y(), leader_trajectory.positions[0].z(), leader_trajectory.headings[0]);
		ROS_INFO("Point_1 avant supression :x=%f, y=%f, z=%f, heading=%f", leader_trajectory.positions[1].x(), leader_trajectory.positions[1].y(), leader_trajectory.positions[1].z(), leader_trajectory.headings[1]);

    leader_trajectory.positions.erase(leader_trajectory.positions.begin());
    leader_trajectory.headings.erase(leader_trajectory.headings.begin());

    ROS_INFO("Taille apres suppression - Positions: %lu, Heading: %lu",
    leader_trajectory.positions.size(), leader_trajectory.headings.size()); 

    ROS_INFO("Point_0 apres supression :x=%f, y=%f, z=%f, heading=%f", leader_trajectory.positions[0].x(), leader_trajectory.positions[0].y(), leader_trajectory.positions[0].z(), leader_trajectory.headings[0]);
    ROS_INFO("Point_1 apres supression :x=%f, y=%f, z=%f, heading=%f", leader_trajectory.positions[1].x(), leader_trajectory.positions[1].y(), leader_trajectory.positions[1].z(), leader_trajectory.headings[1]);
		ROS_INFO("Point_2 apres supression :x=%f, y=%f, z=%f, heading=%f", leader_trajectory.positions[2].x(), leader_trajectory.positions[2].y(), leader_trajectory.positions[2].z(), leader_trajectory.headings[2]);

  }
*/

  double radius = leader_position.norm();
  double alpha  = atan2(leader_position.y(), leader_position.x());
  double offset_rad = 20 * M_PI/180; 

  if (use_estimator) {
    point.position.x() = (radius) * cos (alpha - offset_rad);
    point.position.y() = (radius) * sin (alpha - offset_rad);
    point.position.z() = 3.0;
     
   ROS_INFO("leader position :x=%f ,y=%f", leader_position.x(), leader_position.y());
   ROS_INFO("point :x=%f, y=%f", point.position.x(), point.position.y());

  } else {
    point.position.x() = leader_position.x() + position_offset.x();
    point.position.y() = leader_position.y() + position_offset.y();
    point.position.z() = 3.0;
  }
  point.heading        = leader_heading;
  
  point.use_for_control = true;

  return point;
}
//}

/* createReferenceTrajectory //{ */
ReferenceTrajectory FollowerController::createReferenceTrajectory() {

  // sanity check
  if (!is_initialized || !got_odometry || !got_uvdar || !got_tracker_output) {
    leader_trajectory.positions.push_back(Eigen::Vector3d::Zero());
    leader_trajectory.headings.push_back(0.0);
    leader_trajectory.sampling_time   = 0.0;
    leader_trajectory.use_for_control = false;
    return leader_trajectory;
  }

  // Example - start trajectory at current UAV position and move in the predicted direction of leader motion
  // No subsampling, only two points are created in this example
  Eigen::Vector3d point;
  double          heading;

  leader_trajectory.use_for_control = false;
  if (use_trajectory_reference) {
    if (use_estimator) {
 			 
      point   = leader_position + position_offset;
      heading = leader_heading;

			if ((follower_position_tracker - point).norm() < 10) {

				 leader_trajectory.positions.push_back(point);
     		 leader_trajectory.headings.push_back(heading);
			}

      ROS_INFO("Point :x=%f, y=%f, z=%f, heading=%f", point.x(), point.y(), point.z(), heading);

      leader_trajectory.sampling_time   = control_action_interval;
      leader_trajectory.use_for_control = true;
    }

	  else {
      ROS_WARN("[%s]: Tried to plan a trajectory without leader velocity estimation", ros::this_node::getName().c_str());
    }
  }
/*
  ROS_INFO("Point :x=%f, y=%f, z=%f, heading=%f", leader_trajectory.positions[0].x(), leader_trajectory.positions[0].y(), leader_trajectory.positions[0].z(), leader_trajectory.headings[0]);
  ROS_INFO("Follower_pos :x=%f, y=%f, z=%f, heading=%f", follower_position_tracker.x(), follower_position_tracker.y(), follower_position_tracker.z(), follower_heading_tracker);

  ROS_INFO("Cond_1 =%f, Cond_2 =%f", std::abs(leader_trajectory.positions[0].x() - follower_position_tracker.x()), std::abs(leader_trajectory.positions[0].y() - follower_position_tracker.y()));



  if (!leader_trajectory.positions.empty() && !leader_trajectory.headings.empty() &&
    std::abs(leader_trajectory.positions[0].x() - follower_position_tracker.x()) < epsilon &&
    std::abs(leader_trajectory.positions[0].y() - follower_position_tracker.y()) < epsilon ) {

    ROS_INFO("Taille avant suppression - Positions: %lu, Heading: %lu",
    leader_trajectory.positions.size(), leader_trajectory.headings.size());

    ROS_INFO("Point_0 avant supression :x=%f, y=%f, z=%f, heading=%f", leader_trajectory.positions[0].x(), leader_trajectory.positions[0].y(), leader_trajectory.positions[0].z(), leader_trajectory.headings[0]);
		ROS_INFO("Point_1 avant supression :x=%f, y=%f, z=%f, heading=%f", leader_trajectory.positions[1].x(), leader_trajectory.positions[1].y(), leader_trajectory.positions[1].z(), leader_trajectory.headings[1]);

    leader_trajectory.positions.erase(leader_trajectory.positions.begin());
    leader_trajectory.headings.erase(leader_trajectory.headings.begin());

    ROS_INFO("Taille apres suppression - Positions: %lu, Heading: %lu", leader_trajectory.positions.size(), leader_trajectory.headings.size()); 

    ROS_INFO("Point_0 apres supression :x=%f, y=%f, z=%f, heading=%f", leader_trajectory.positions[0].x(), leader_trajectory.positions[0].y(), leader_trajectory.positions[0].z(), leader_trajectory.headings[0]);
    ROS_INFO("Point_1 apres supression :x=%f, y=%f, z=%f, heading=%f", leader_trajectory.positions[1].x(), leader_trajectory.positions[1].y(), leader_trajectory.positions[1].z(), leader_trajectory.headings[1]);
		ROS_INFO("Point_2 apres supression :x=%f, y=%f, z=%f, heading=%f", leader_trajectory.positions[2].x(), leader_trajectory.positions[2].y(), leader_trajectory.positions[2].z(), leader_trajectory.headings[2]);

  }
*/
  return leader_trajectory;
}
//}

/* createVelocityReference //{ */
VelocityReference FollowerController::createVelocityReference() {
  VelocityReference velocity_cmd;

  if (!got_odometry || !got_uvdar || !got_tracker_output) {
    velocity_cmd.velocity        = Eigen::Vector3d(0, 0, 0);
    velocity_cmd.heading         = 0;
    velocity_cmd.height          = 0;
    velocity_cmd.use_for_control = false;
  }

  if (use_estimator) {
    velocity_cmd.velocity = leader_predicted_velocity;
    velocity_cmd.height   = leader_predicted_position.z() + position_offset.z();
    velocity_cmd.heading  = follower_heading_tracker;
  }

  if (use_velocity_reference) {
    velocity_cmd.use_for_control = true;
  } else {
    velocity_cmd.use_for_control = false;
  }

  return velocity_cmd;

 /* ROS_INFO("command.velocity : x=%f, y=%f, z=%f", command.velocity.x(), command.velocity.y(), command.velocity.z());
  ROS_INFO("norm velocity : %f", command.velocity.norm());
  */

}
//}

/* getCurrentEstimate //{ */

// You can use this method for debugging purposes.
// It allows you to visualize the leader predictions in rviz
// It is called once per control action of the summer_schoo_supervisor

nav_msgs::Odometry FollowerController::getCurrentEstimate() {
  nav_msgs::Odometry leader_est;

  if (use_estimator) {
    auto leader_prediction          = estimator.predict(Eigen::Vector3d(0.0, 0.0, 0.0), control_action_interval);
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
