#pragma once

#include <mc_tasks/PostureTask.h>
#include <RBDyn/FD.h>

namespace mc_tasks
{

/**
 * TorqueTask: Specify desired joint torques, injects the corresponding reference acceleration into a PostureTask
 * backend.
 * TODO: add mimics handling
 */
struct MC_TASKS_DLLAPI TorqueTask : public PostureTask
{
public:
  TorqueTask(const mc_solver::QPSolver & solver, unsigned int rIndex, double weight = 1000.0);

  void torque(const std::vector<std::vector<double>> & tau);
  void torque(const std::string & jointName, std::vector<double> tau);
  std::vector<std::vector<double>> torque() const;

  /** Set specific joint targets
   *
   * \param joints Map of joint's name to joint's configuration
   *
   */
  void target(const std::map<std::string, std::vector<double>> & joints);

  Eigen::VectorXd jointsToDofs(const std::vector<std::vector<double>> & joints);

  void reset() override;
  void update(mc_solver::QPSolver & solver) override;
  void addToGUI(mc_rtc::gui::StateBuilder &) override;
  void addToLogger(mc_rtc::Logger & logger) override;

private:
  std::vector<std::vector<double>> torque_;
  double dt_;
  unsigned int rIndex_;
  const mc_rbdyn::Robots & robots_;
};

} // namespace mc_tasks
