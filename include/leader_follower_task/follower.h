#ifndef LEADER_FOLLOWER_TASK__FOLLOWER_H
#define LEADER_FOLLOWER_TASK__FOLLOWER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/TrackerCommand.h>
#include <mrs_msgs/SpeedTrackerCommand.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/String.h>
#include <dynamic_reconfigure/server.h>

#include <eigen3/Eigen/Core>

#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/param_loader.h>

#include <leader_follower_task/FollowerConfig.h>

struct ReferencePoint
{
  Eigen::Vector3d position;
  double          heading;
  bool            use_for_control;
};

struct ReferenceTrajectory
{
  std::vector<Eigen::Vector3d> positions;
  std::vector<double>          headings;
  double                       sampling_time;
  bool                         use_for_control;
};

struct SpeedCommand
{
  Eigen::Vector3d velocity;
  double          heading;
  double          height;
  bool            use_for_control;
};

class FollowerController {

public:
  FollowerController() {
  }

  ~FollowerController() {
  }

  ReferencePoint      createReferencePoint();
  ReferenceTrajectory createReferenceTrajectory();
  SpeedCommand        createSpeedCommand();

  leader_follower_task::FollowerConfig initialize(mrs_lib::ParamLoader& param_loader);

  void receiveOdometry(const nav_msgs::Odometry& odometry_msg);
  void receiveTrackerOutput(const mrs_msgs::TrackerCommand& position_cmd);
  void receiveUvdar(const geometry_msgs::PoseWithCovarianceStamped& uvdar_msg);
  void dynamicReconfigureCallback(leader_follower_task::FollowerConfig& config, uint32_t level);

  nav_msgs::Odometry getCurrentEstimate();
  double             getControlActionInterval() {
    return control_action_interval;
  }

private:
  double control_action_interval = 0.0;
};

#endif
