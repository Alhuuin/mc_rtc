/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TorqueTask.h>
#include <Eigen/src/Core/Matrix.h>

namespace mc_tasks
{
/*! \brief Joint-space PD task in torque control.
 *
 * The TorquePDJointTask computes desired joint torques using a proportional–
 * derivative (PD) control law in joint space. It relies on TorqueTask to find
 * the joint accelerations that minimize the error between the commanded joint
 * torques and the torques predicted by the robot dynamic model, while
 * respecting the constraints of the controller. As such, this task is intended
 * to be used in torque-control mode only.
 *
 * The commanded torque is computed as:
 *
 *   \f[
 *     \tau = K_p (q_d - q) + K_d (\dot{q}_d - \dot{q})
 *   \f]
 *
 * where \f$q\f$ and \f$\dot{q}\f$ are the current joint position and velocity,
 * and \f$q_d\f$ and \f$\dot{q}_d\f$ are the desired joint position and velocity.
 *
 * Additional torque components can be added to the command:
 *  - feedforward torques \f$\tau_{ff}\f$,
 *  - external torque compensation \f$\tau_{ext}\f$,
 *  - gravity compensation \f$\tau_{g}\f$.
 *
 * By default, all additional torque components are set to zero.
 *
 * The default targets are the current joint configuration for position
 * (\f$q_d = q\f$) and zero velocity for the desired velocity
 * (\f$\dot{q}_d = 0\f$).
 *
 */
struct MC_TASKS_DLLAPI TorquePDJointTask : public TorqueTask
{
public:
  TorquePDJointTask(const mc_solver::QPSolver & solver, unsigned int rIndex, double stiffness, double weight);
  void setStiffness(double stiffness);
  void setDamping(double damping);
  void setStiffness(const Eigen::VectorXd & stiffness);
  void setDamping(const Eigen::VectorXd & damping);
  void setPosTarget(const Eigen::VectorXd & qd);
  void setVelTarget(const Eigen::VectorXd & qd_dot);
  void setTorqueFeedforward(const Eigen::VectorXd & tau_ff);

  const Eigen::VectorXd & stiffness() const;
  const Eigen::VectorXd & damping() const;
  const Eigen::VectorXd & posTarget() const;
  const Eigen::VectorXd & velTarget() const;
  const Eigen::VectorXd & torqueFeedforward() const;

protected:
  void update(mc_solver::QPSolver & solver);
  void addToGUI(mc_rtc::gui::StateBuilder & gui);
  void addToLogger(mc_rtc::Logger & logger);

  /** Robot handled by the task */
  const mc_rbdyn::Robots & robots_;
  unsigned int rIndex_;

  const int nbActuatedJoints; // Number of actuated joints (excluding floating base)

  Eigen::VectorXd stiffness_; // Kp
  Eigen::VectorXd damping_; // Kd

  Eigen::VectorXd posTarget_; // qd
  Eigen::VectorXd velTarget_; // qd_dot
  Eigen::VectorXd torqueFeedforward_; // tau_ff

  Eigen::VectorXd posError_;
  Eigen::VectorXd velError_;

  Eigen::VectorXd torque_target_;
};

} // namespace mc_tasks
