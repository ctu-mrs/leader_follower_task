#ifndef LEADER_FOLLOWER_TASK__MESSAGE_UTILS_H
#define LEADER_FOLLOWER_TASK__MESSAGE_UTILS_H

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/SpeedTrackerCommand.h>
#include <mrs_msgs/ReferenceStamped.h>

/**
 * @brief Create a reference point message for the MPC tracker
 *
 * @param position - desired position of the UAV in a specified reference frame
 * @param heading - desired 'forward' direction of the drone's body, aka how much is the body X-axis rotated with respect to the reference frame
 * @param frame - name of the frame of reference
 *
 * @return message to be published into the control_manager/reference topic
 */
mrs_msgs::ReferenceStamped buildMpcReference(Eigen::Vector3d position, double heading, std::string frame);

/**
 * @brief Create a reference point message for the MPC tracker. Do not change UAV heading
 *
 * @param position - desired position of the UAV in a specified reference frame
 * @param frame - name of the frame of reference
 *
 * @return message to be published into the /control_manager/reference topic
 */
mrs_msgs::ReferenceStamped buildMpcReference(Eigen::Vector3d desired_position, std::string frame);

/**
 * @brief Create a reference trajectory for the MPC tracker
 *
 * RECOMMENDED READING if you wish to use trajectory reference: https://ctu-mrs.github.io/mrs_msgs/msg/TrajectoryReference.html
 *
 * @param position - series of positions along the trajectory
 * @param heading - series of headings in corresponding positions
 * @param sampling_time - time step between the provided samples
 * @param frame - name of the frame of reference
 *
 * @return message to be published into the /control_manager/trajectory_reference topic
 */
mrs_msgs::TrajectoryReference buildMpcTrajectoryReference(std::vector<Eigen::Vector3d> positions, std::vector<double> headings, double sampling_time,
                                                          std::string frame);

/**
 * @brief Create a reference trajectory for the MPC tracker. Do not change heading
 *
 * @param position - series of positions along the trajectory
 * @param sampling_time - time step between the provided samples
 * @param frame - name of the frame of reference
 *
 * @return message to be published into the /control_manager/trajectory_reference topic
 */
mrs_msgs::TrajectoryReference buildMpcTrajectoryReference(std::vector<Eigen::Vector3d> position, double sampling_time, std::string frame);

/**
 * @brief Create a command for the SpeedTracker
 *
 * @param velocity - desired velocity
 * @param heading - desired heading
 * @param height - desired height to be maintained
 * @param frame - name of the frame of reference
 *
 * @return message to be published into the /control_manager/speed_tracker/command
 */
mrs_msgs::SpeedTrackerCommand buildSpeedTrackerCommand(Eigen::Vector3d velocity, double heading, double height, std::string frame);


/**
 * @brief Create an odometry message using a position and velocity
 * Primary usage of this function is debugging and logging
 *
 * @param position
 * @param velocity
 * @param frame
 *
 * @return odometry message
 */
nav_msgs::Odometry buildOdometryMessage(Eigen::Vector3d position, Eigen::Vector3d velocity, std::string frame);

#endif
