#pragma once

#include <mc_tasks/PostureTask.h>
#include <RBDyn/FD.h>

namespace mc_tasks
{

/**
 * TorqueTask: Specify desired joint torques, injects the corresponding reference acceleration into a PostureTask
 * backend.
 */
struct MC_TASKS_DLLAPI TorqueTask : public PostureTask
{
public:
  TorqueTask(const mc_solver::QPSolver & solver, unsigned int rIndex, double weight = 1000.0);

  //! Set the desired joint torques (size: nrDof)
  void desiredTorque(const Eigen::VectorXd & tau);
  //! Get the current desired joint torques
  const Eigen::VectorXd & desiredTorque() const;

  /*! \brief Get the current joint torques */
  Eigen::VectorXd currentTorques() const;

  void reset() override;
  void update(mc_solver::QPSolver & solver) override;
  void addToGUI(mc_rtc::gui::StateBuilder &) override;
  void addToLogger(mc_rtc::Logger & logger) override;

private:
  Eigen::VectorXd desiredTorque_;
  double dt_;
  unsigned int rIndex_;
  const mc_rbdyn::Robots & robots_;
};

} // namespace mc_tasks
