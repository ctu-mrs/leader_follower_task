#ifndef LEADER_FOLLOWER_TASK__VELOCITY_ESTIMATOR_H
#define LEADER_FOLLOWER_TASK__VELOCITY_ESTIMATOR_H

#include <eigen3/Eigen/Core>
#include <mrs_lib/lkf.h>
#include <ros/ros.h>

class VelocityEstimator {

public:
  using kalman3D = mrs_lib::LKF<6, 3, 3>;

private:
  std::unique_ptr<kalman3D> velocity_estimator_;

  kalman3D::statecov_t sc;

  kalman3D::A_t A;
  kalman3D::B_t B;
  kalman3D::H_t H;
  kalman3D::R_t R;
  kalman3D::Q_t Q;
  kalman3D::P_t P;

  double dt;

public:
  VelocityEstimator();

  /**
   * @brief Construct the system matrices and initialize the estimator
   *
   * @param Q - process noise matrix (6x6)
   * @param R - measurement noise matrix (3x3)
   * @param x0 - initial state (x,y,z,dx,dy,dz)
   * @param dt - time step between iterations
   */
  VelocityEstimator(kalman3D::Q_t Q, kalman3D::R_t R, kalman3D::x_t inital_states, double dt);

  /**
   * @brief Perform the prediction step of the Kalman filter
   *
   * @param velocity_estimate - measured input to the system
   * @param dt_since_last_prediction - time interval between predictions, since the system matrices are dependent on it and the message rate may not be constant
   *
   * @return Vector6 (x,y,z,dx,dy,dz) system states after the prediction update
   */
  kalman3D::x_t predict(Eigen::Vector3d velocity_estimate, double dt_since_last_prediction);

  /**
   * @brief  Perform the fusion (correction) step of the Kalman filter.
   * Incorporates new observations of the system state into the model.
   *
   * @param position_measurement - measurement (observation) of the system state
   *
   * @return Vector6 (x,y,z,dx,dy,dz) system states after the fusion
   */
  kalman3D::x_t fuse(Eigen::Vector3d position_measurement);

  /**
   * @brief Get a current vector of all system states
   *
   * @return Vector6 system states (x,y,z,dx,dy,dz)
   */
  kalman3D::x_t getStates();

  /**
   * @brief Get a covariance matrix of the system states
   *
   * @return Matrix6x6
   */
  kalman3D::P_t getCovariance();

  /**
   * @brief Get only the velocity part of the system states
   *
   * @return Vector3 (dx,dy,dz)
   */
  Eigen::Vector3d getVelocity();

  /**
   * @brief Get only the position part of the system states
   *
   * @return Vector3 (x,y,z)
   */
  Eigen::Vector3d getPosition();
};

#endif
