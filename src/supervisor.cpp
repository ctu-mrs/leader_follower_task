#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int64.h>
#include <std_srvs/Trigger.h>
#include <uvdar_core/ImagePointsWithFloatStamped.h>
#include <dynamic_reconfigure/server.h>

#include <eigen3/Eigen/Core>

#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/param_loader.h>

#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/SpeedTrackerCommand.h>
#include <mrs_msgs/TrackerCommand.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>

#include <leader_follower_task/message_utils.h>
#include <leader_follower_task/follower.h>
#include <leader_follower_task/FollowerConfig.h>

#define MAX_ERRONEOUS_COMMANDS_COUNT 10

#define MIN_COMMAND_HEIGHT 2.0               // [m]
#define MAX_COMMAND_HEIGHT 4.0               // [m]
#define MAX_COMMAND_DISTANCE_THRESHOLD 15.0  // [m]
#define MAX_VELOCITY_MAGNITUDE 5.0           // [m/s]

bool initialized         = false;
bool using_speed_tracker = false;
bool contact_broken      = false;
bool counting_score      = false;

double visual_contact_timeout  = 1.7;   // [s]
double command_timeout         = 1.0;   // [s]
double score_timer_interval    = 0.01;  // [s]
double control_action_interval = 0.0;   // [s]

ros::Time last_contact_time;
ros::Time last_command_time;
int       score                    = 0;
int       left_blinkers            = 0;
int       right_blinkers           = 0;
int       erroneous_commands_count = 0;

Eigen::Vector3d leader_raw_pos_uvdar;
Eigen::Vector3d leader_raw_pos_odom;

ros::Subscriber odometry_subscriber;
ros::Subscriber position_cmd_subscriber;
ros::Subscriber uvdar_subscriber;
ros::Subscriber left_blinkers_subscriber;
ros::Subscriber right_blinkers_subscriber;
ros::Subscriber leader_odometry_subscriber;

ros::Publisher score_publisher;
ros::Publisher leader_raw_pos_publisher;
ros::Publisher leader_estim_pos_publisher;

ros::Timer score_timer, control_action_timer;

ros::ServiceClient switch_tracker_client;
ros::ServiceServer start_score_counting_server;
ros::Publisher     mpc_reference_publisher;
ros::Publisher     mpc_trajectory_publisher;
ros::Publisher     speed_tracker_command_publisher;

Eigen::Vector3d follower_pos_odom;
Eigen::Vector3d follower_pos_cmd;

FollowerController fc;
std::string        uav_frame;

/* switchToSpeedTracker //{ */
void switchToSpeedTracker() {
  if (using_speed_tracker) {
    return;
  }
  mrs_msgs::String msg;
  msg.request.value = "SpeedTracker";
  switch_tracker_client.call(msg.request, msg.response);
  if (!msg.response.success) {
    ROS_WARN("[%s]: %s", ros::this_node::getName().c_str(), msg.response.message.c_str());
  } else {
    ROS_INFO("[%s]: Switched to SpeedTracker", ros::this_node::getName().c_str());
    using_speed_tracker = true;
  }
}
//}

/* switchToMpcTracker //{ */
void switchToMpcTracker() {
  if (!using_speed_tracker) {
    return;
  }
  mrs_msgs::String msg;
  msg.request.value = "MpcTracker";
  switch_tracker_client.call(msg.request, msg.response);
  if (!msg.response.success) {
    ROS_WARN("[%s]: %s", ros::this_node::getName().c_str(), msg.response.message.c_str());
  } else {
    ROS_INFO("[%s]: Switched to MPC tracker", ros::this_node::getName().c_str());
    using_speed_tracker = false;
  }
}
//}

/* publishLeaderRawPos //{ */
void publishLeaderRawPos() {
  nav_msgs::Odometry leader_raw_pos;
  leader_raw_pos.header.frame_id      = uav_frame;
  leader_raw_pos.header.stamp         = ros::Time::now();
  leader_raw_pos.pose.pose.position.x = leader_raw_pos_uvdar.x();
  leader_raw_pos.pose.pose.position.y = leader_raw_pos_uvdar.y();
  leader_raw_pos.pose.pose.position.z = leader_raw_pos_uvdar.z();
  leader_raw_pos_publisher.publish(leader_raw_pos);
}
//}

/* publishLeaderFilteredPos //{ */
void publishLeaderFilteredPos() {
  auto leader_estim_pos            = fc.getCurrentEstimate();
  leader_estim_pos.header.frame_id = uav_frame;
  leader_estim_pos.header.stamp    = ros::Time::now();
  leader_estim_pos_publisher.publish(leader_estim_pos);
}
//}

/* scoreTimer //{ */
void scoreTimer(const ros::TimerEvent /* &event */) {

  if ((left_blinkers + right_blinkers) >= 2) {
    last_contact_time = ros::Time::now();
  }

  auto now = ros::Time::now().toSec();

  // check if visual contact is broken
  double visual_dt = now - last_contact_time.toSec();
  if (visual_dt > visual_contact_timeout) {
    ROS_ERROR("[%s]: Visual contact broken for longer than %.2f sec! Following terminated!", ros::this_node::getName().c_str(), visual_contact_timeout);
    contact_broken = true;
  }

  // check if control commands are comming in
  double command_dt = now - last_command_time.toSec();
  if (command_dt > command_timeout) {
    ROS_ERROR("[%s]: Valid control command was not provided for longer than %.2f sec! Following terminated!", ros::this_node::getName().c_str(),
              command_timeout);
  }

  if (erroneous_commands_count > MAX_ERRONEOUS_COMMANDS_COUNT) {
    ROS_ERROR("[%s]: Too many erroneous commands received! Following terminated!", ros::this_node::getName().c_str());
    contact_broken = true;
  }

  if (counting_score) {
    score++;
    std_msgs::Int64 score_msg;
    score_msg.data = score;
    score_publisher.publish(score_msg);
  }
}
//}

/* controlAction //{ */
void controlAction(const ros::TimerEvent /* &event */) {

  publishLeaderRawPos();
  publishLeaderFilteredPos();

  // call student's createSpeedCommand
  auto speed_command_request = fc.createSpeedCommand();
  auto speed_command         = buildSpeedTrackerCommand(speed_command_request.velocity, speed_command_request.heading, speed_command_request.height, uav_frame);
  speed_tracker_command_publisher.publish(speed_command);

  if (sqrt(pow(follower_pos_odom.x() - leader_raw_pos_odom.x(), 2) + pow(follower_pos_odom.y() - leader_raw_pos_odom.y(), 2)) > 15.0) {
    ROS_WARN("[%s]: UAVs separation limit exceeded!", ros::this_node::getName().c_str());
    erroneous_commands_count++;
    return;
  }

  if (speed_command_request.use_for_control) {

    /* speed command sanity checks //{ */

    double uav_separation = (leader_raw_pos_odom - follower_pos_odom).norm();
    if (uav_separation < 3.0) {
      ROS_WARN("[%s]: UAVs are too close together! SpeedTracker cannot be used for control!", ros::this_node::getName().c_str());
      erroneous_commands_count++;
      switchToMpcTracker();
      return;
    }

    if (speed_command_request.height < MIN_COMMAND_HEIGHT) {
      ROS_WARN("[%s]: Reference point set too low! The command will be discarded", ros::this_node::getName().c_str());
      erroneous_commands_count++;
      return;
    }
    if (speed_command_request.height > MAX_COMMAND_HEIGHT) {
      ROS_WARN("[%s]: Reference point set too high! The command will be discarded", ros::this_node::getName().c_str());
      erroneous_commands_count++;
      return;
    }
    if (speed_command_request.velocity.norm() > MAX_VELOCITY_MAGNITUDE) {
      ROS_WARN("[%s]: Requested velocity is too high! The command will be discarded", ros::this_node::getName().c_str());
      erroneous_commands_count++;
      return;
    }
    //}

    erroneous_commands_count = 0;

    switchToSpeedTracker();
    last_command_time = speed_command.header.stamp;
  } else {
    switchToMpcTracker();

    // call student's createReferenceTrajectory
    auto reference_trajectory_request = fc.createReferenceTrajectory();
    if (reference_trajectory_request.use_for_control) {

      /* reference trajectory sanity checks //{ */

      if (reference_trajectory_request.positions.size() != reference_trajectory_request.headings.size()) {
        ROS_WARN("[%s]: Number of trajectory points (%ld) does not match the number of trajectory headings (%ld)! The trajectory will be discarded",
                 ros::this_node::getName().c_str(), reference_trajectory_request.positions.size(), reference_trajectory_request.headings.size());
        erroneous_commands_count++;
        return;
      }

      for (size_t i = 0; i < reference_trajectory_request.positions.size(); i++) {

        if (reference_trajectory_request.positions[i].z() < MIN_COMMAND_HEIGHT) {
          ROS_WARN("[%s]: Trajectory reference point (%ld) set too low! The trajectory will be discarded", ros::this_node::getName().c_str(), i);
          erroneous_commands_count++;
          return;
        }

        if (reference_trajectory_request.positions[i].z() > MAX_COMMAND_HEIGHT) {
          ROS_WARN("[%s]: Trajectory reference point (%ld) set too high! The trajectory will be discarded", ros::this_node::getName().c_str(), i);
          erroneous_commands_count++;
          return;
        }

        if ((reference_trajectory_request.positions[0] - follower_pos_odom).norm() > MAX_COMMAND_DISTANCE_THRESHOLD) {
          ROS_WARN("[%s]: Trajectory start [%.2f, %.2f, %.2f] set too far from the UAV position [%.2f, %.2f, %.2f]. The trajectory will be discarded",
                   ros::this_node::getName().c_str(), reference_trajectory_request.positions[0].x(), reference_trajectory_request.positions[0].y(),
                   reference_trajectory_request.positions[0].z(), follower_pos_odom.x(), follower_pos_odom.y(), follower_pos_odom.z());
          erroneous_commands_count++;
          return;
        }
      }
      //} reference trajectory sanity checks

      erroneous_commands_count = 0;

      auto reference_trajectory = buildMpcTrajectoryReference(reference_trajectory_request.positions, reference_trajectory_request.headings,
                                                              reference_trajectory_request.sampling_time, uav_frame);
      mpc_trajectory_publisher.publish(reference_trajectory);
      last_command_time = reference_trajectory.header.stamp;

    } else {

      // call student's createReferencePoint
      auto reference_point_request = fc.createReferencePoint();

      /* reference command sanity checks //{ */
      if (reference_point_request.position.z() < MIN_COMMAND_HEIGHT) {
        ROS_WARN("[%s]: Reference point set too low! The command will be discarded", ros::this_node::getName().c_str());
        erroneous_commands_count++;
        return;
      }

      if (reference_point_request.position.z() > MAX_COMMAND_HEIGHT) {
        ROS_WARN("[%s]: Reference point set too high! The command will be discarded", ros::this_node::getName().c_str());
        erroneous_commands_count++;
        return;
      }

      if ((reference_point_request.position - follower_pos_odom).norm() > MAX_COMMAND_DISTANCE_THRESHOLD) {
        ROS_WARN("[%s]: Reference point [%.2f, %.2f, %.2f] set too far from the UAV position [%.2f, %.2f, %.2f]. The command will be discarded",
                 ros::this_node::getName().c_str(), reference_point_request.position.x(), reference_point_request.position.y(),
                 reference_point_request.position.z(), follower_pos_odom.x(), follower_pos_odom.y(), follower_pos_odom.z());
        erroneous_commands_count++;
        return;
      }
      //} reference command sanity check

      erroneous_commands_count = 0;

      auto reference_point = buildMpcReference(reference_point_request.position, reference_point_request.heading, uav_frame);
      mpc_reference_publisher.publish(reference_point);
      last_command_time = reference_point.header.stamp;
    }
  }
}
//}

/* leftBlinkersCallback //{ */
void leftBlinkersCallback(const uvdar_core::ImagePointsWithFloatStamped& msg) {
  left_blinkers = msg.points.size();
}
//}

/* rightBlinkersCallback //{ */
void rightBlinkersCallback(const uvdar_core::ImagePointsWithFloatStamped& msg) {
  right_blinkers = msg.points.size();
}
//}

/* uvdarCallback //{ */
void uvdarCallback(const mrs_msgs::PoseWithCovarianceArrayStamped& uvdar_msg) {
  if (!initialized) {
    return;
  }
  if (uvdar_msg.poses.size() < 1) {
    return;
  }
  geometry_msgs::PoseWithCovarianceStamped msg;
  msg.header           = uvdar_msg.header;
  msg.pose.pose        = uvdar_msg.poses[0].pose;
  leader_raw_pos_uvdar = Eigen::Vector3d(uvdar_msg.poses[0].pose.position.x, uvdar_msg.poses[0].pose.position.y, uvdar_msg.poses[0].pose.position.z);
  fc.receiveUvdar(msg);
}
//}

/* odometryCallback //{ */
void odometryCallback(const nav_msgs::Odometry& odometry_msg) {
  if (!initialized) {
    return;
  }
  follower_pos_odom = Eigen::Vector3d(odometry_msg.pose.pose.position.x, odometry_msg.pose.pose.position.y, odometry_msg.pose.pose.position.z);
  fc.receiveOdometry(odometry_msg);
}
//}

/* leaderOdometryCallback //{ */
void leaderOdometryCallback(const nav_msgs::Odometry& odometry_msg) {
  if (!initialized) {
    return;
  }
  leader_raw_pos_odom = Eigen::Vector3d(odometry_msg.pose.pose.position.x, odometry_msg.pose.pose.position.y, odometry_msg.pose.pose.position.z);
}
//}

/* positionCmdCallback //{ */
void positionCmdCallback(const mrs_msgs::TrackerCommand& position_cmd) {
  if (!initialized) {
    return;
  }
  follower_pos_cmd = Eigen::Vector3d(position_cmd.position.x, position_cmd.position.y, position_cmd.position.z);

  fc.receiveTrackerOutput(position_cmd);
}
//}

/* startScoreCountingCallback //{ */
bool startScoreCountingCallback([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  if (counting_score) {
    res.success = false;
    res.message = "Already counting score!";
    return false;
  }
  counting_score = true;
  res.success    = true;
  return true;
}
//}

/* main //{ */
int main(int argc, char** argv) {

  ros::init(argc, argv, "supervisor");
  ros::NodeHandle nh = ros::NodeHandle("~");

  std::string       reference_frame;
  std::stringstream ss;
  ss << std::getenv("UAV_NAME") << "/world_origin";
  uav_frame = ss.str();

  odometry_subscriber        = nh.subscribe("odometry_in", 10, &odometryCallback);
  position_cmd_subscriber    = nh.subscribe("position_cmd_in", 10, &positionCmdCallback);
  uvdar_subscriber           = nh.subscribe("uvdar_in", 10, &uvdarCallback);
  left_blinkers_subscriber   = nh.subscribe("left_blinkers_in", 10, &leftBlinkersCallback);
  right_blinkers_subscriber  = nh.subscribe("right_blinkers_in", 10, &rightBlinkersCallback);
  leader_odometry_subscriber = nh.subscribe("leader_odometry_in", 10, &leaderOdometryCallback);

  score_publisher                 = nh.advertise<std_msgs::Int64>("score_out", 1);
  mpc_reference_publisher         = nh.advertise<mrs_msgs::ReferenceStamped>("reference_point_out", 1);
  mpc_trajectory_publisher        = nh.advertise<mrs_msgs::TrajectoryReference>("reference_trajectory_out", 1);
  speed_tracker_command_publisher = nh.advertise<mrs_msgs::SpeedTrackerCommand>("speed_tracker_command_out", 1);
  switch_tracker_client           = nh.serviceClient<mrs_msgs::String>("switch_tracker_srv_out");
  start_score_counting_server     = nh.advertiseService("start_score_counting_in", &startScoreCountingCallback);

  leader_raw_pos_publisher   = nh.advertise<nav_msgs::Odometry>("leader_raw_pos_out", 1);
  leader_estim_pos_publisher = nh.advertise<nav_msgs::Odometry>("leader_estim_pos_out", 1);

  dynamic_reconfigure::Server<leader_follower_task::FollowerConfig>               dynamic_reconfigure_server;
  dynamic_reconfigure::Server<leader_follower_task::FollowerConfig>::CallbackType dynamic_reconfigure_callback_t;

  ROS_INFO("[%s]: Waiting for position from UVDAR...", ros::this_node::getName().c_str());
  while (ros::ok()) {
    if (left_blinkers + right_blinkers > 1) {
      last_contact_time = ros::Time::now();
      break;
    }
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }
  mrs_lib::ParamLoader param_loader(nh, "follower");
  initialized    = true;
  auto dr_config = fc.initialize(param_loader);
  dynamic_reconfigure_server.updateConfig(dr_config);
  dynamic_reconfigure_callback_t = boost::bind(&FollowerController::dynamicReconfigureCallback, fc, _1, _2);
  dynamic_reconfigure_server.setCallback(dynamic_reconfigure_callback_t);
  ROS_INFO("[%s]: Initialization complete", ros::this_node::getName().c_str());

  control_action_interval = fc.getControlActionInterval();

  if (control_action_interval < 0.01) {
    ROS_WARN("[%s]: Cannot set shorter control interval than 0.01 seconds! Value will be truncated to 0.01 seconds.", ros::this_node::getName().c_str());
  }

  ROS_INFO("[%s]: Starting control action loop at %.3f Hz", ros::this_node::getName().c_str(), (1.0 / control_action_interval));
  last_command_time = ros::Time::now();

  score_timer          = nh.createTimer(ros::Duration(score_timer_interval), &scoreTimer);
  control_action_timer = nh.createTimer(ros::Duration(control_action_interval), &controlAction);

  while (ros::ok() && !contact_broken) {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.01));
  }

  control_action_timer.stop();
  score_timer.stop();
  switchToMpcTracker();
  ROS_INFO("[%s]: Final score: %d", ros::this_node::getName().c_str(), score);

  return 0;
}
//}
