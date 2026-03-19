/*
 * Copyright 2015-2026 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TorqueTask.h>
#include <SpaceVecAlg/MotionVec.h>
#include <Eigen/src/Core/Matrix.h>

namespace mc_tasks
{
/*! \brief Cartesian-space PD task relative to a body, in torque control.
 *
 * The TorquePDRelativeCartesianTask computes desired joint torques using a proportional–
 * derivative (PD) control law in cartesian space. It relies on TorqueTask to find
 * the joint accelerations that minimize the error between the commanded joint
 * torques and the torques predicted by the robot dynamic model, while
 * respecting the constraints of the controller. As such, this task is intended
 * to be used in torque-control mode only.
 *
 * The commanded torque is computed as:
 *
 *   \f[
 *     \tau = J^T[K_p (x_d - x) + K_d (\dot{x}_d - \dot{x})]
 *   \f]
 *
 * where \f$x\f$ and \f$\dot{x}\f$ are the current cartesian position and velocity,
 * and \f$x_d\f$ and \f$\dot{x}_d\f$ are the desired cartesian position and velocity.
 * Cartesian position and velocity are expressed in the relative frame, and the Jacobian is the geometric Jacobian
 * expressed in the relative frame.
 *
 * Additional torque components can be added to the command:
 *  - feedforward torques \f$\tau_{ff}\f$,
 *  - external torque compensation \f$\tau_{ext}\f$,
 *  - gravity compensation \f$\tau_{g}\f$.
 *
 * By default, all additional torque components are set to zero.
 *
 * The default targets are the current cartesian configuration for position
 * (\f$x_d = x\f$) and zero velocity for the desired velocity
 * (\f$\dot{x}_d = 0\f$).
 *
 */
struct MC_TASKS_DLLAPI TorquePDRelativeCartesianTask : public TorqueTask
{
public:
  /*! \brief Constructor
   *
   * \param solver QP solver instance
   *
   * \param frame Frame controlled by this task
   *
   * \param relative Relative frame for the task.
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   */
  TorquePDRelativeCartesianTask(const mc_solver::QPSolver & solver,
                                const mc_rbdyn::RobotFrame & frame,
                                const mc_rbdyn::RobotFrame & relative,
                                double stiffness = 100.0,
                                double weight = 500.0);

  /*! \brief Constructor
   *
   * \param solver QP solver instance
   *
   * \param bodyName Name of the body to control
   *
   * \param relBodyName Name of the body relatively to which the end-effector
   * is controlled. If empty, defaults to the robot's base.
   *
   * \param robots Robots controlled by this task
   *
   * \param rIndex Index of the robot controlled by this task
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   */
  TorquePDRelativeCartesianTask(const mc_solver::QPSolver & solver,
                                const std::string & bodyName,
                                const std::string & relBodyName,
                                const mc_rbdyn::Robots & robots,
                                unsigned int rIndex,
                                double stiffness = 100.0,
                                double weight = 500.0);

  /*! \brief Constructor that defaults to the robot's base as relative frame
   *
   * \param solver QP solver instance
   *
   * \param bodyName Name of the body to control
   *
   * \param robots Robots controlled by this task
   *
   * \param rIndex Index of the robot controlled by this task
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   */
  TorquePDRelativeCartesianTask(const mc_solver::QPSolver & solver,
                                const std::string & bodyName,
                                const mc_rbdyn::Robots & robots,
                                unsigned int rIndex,
                                double stiffness = 100.0,
                                double weight = 500.0);

  void reset() override;

  void setStiffness(double stiffness);
  void setDamping(double damping);
  void setStiffness(const Eigen::Vector6d & stiffness);
  void setDamping(const Eigen::Vector6d & damping);
  void setPosTarget(const sva::PTransformd & xd);
  void setVelTarget(const sva::MotionVecd & xd_dot);
  void setTorqueFeedforward(const Eigen::VectorXd & tau_ff);

  const Eigen::Vector6d & stiffness() const;
  const Eigen::Vector6d & damping() const;
  const sva::PTransformd & posTarget() const;
  const sva::MotionVecd & velTarget() const;
  const Eigen::VectorXd & torqueFeedforward() const;

protected:
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;
  void addToLogger(mc_rtc::Logger & logger) override;
  const mc_rbdyn::ConstRobotFramePtr & frame() const { return frame_; }

private:
  void update(mc_solver::QPSolver & solver) override;

  /** Robot handled by the task */
  const mc_rbdyn::Robots & robots_;
  unsigned int rIndex_;

  const int nbActuatedJoints; // Number of actuated joints (excluding floating base)

  Eigen::Vector6d stiffness_; // Kp
  Eigen::Vector6d damping_; // Kd

  sva::PTransformd posTarget_; // xd
  sva::MotionVecd velTarget_; // xd_dot
  Eigen::VectorXd torqueFeedforward_; // tau_ff

  sva::MotionVecd posError_;
  sva::MotionVecd velError_;

  Eigen::VectorXd torqueTarget_;
  mc_rbdyn::ConstRobotFramePtr frame_;
  mc_rbdyn::ConstRobotFramePtr relative_;
};

} // namespace mc_tasks
