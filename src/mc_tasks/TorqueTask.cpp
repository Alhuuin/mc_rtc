#include <mc_rbdyn/Robot.h>
#include <mc_rtc/gui/ArrayLabel.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/log/Logger.h>
#include <mc_solver/QPSolver.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/TorqueTask.h>
#include <RBDyn/FD.h>
#include "mc_rtc/logging.h"
#include <cstddef>

namespace mc_tasks
{

TorqueTask::TorqueTask(const mc_solver::QPSolver & solver, unsigned int rIndex, double weight)
: PostureTask(solver, rIndex, 0, weight), dt_(solver.dt()), rIndex_(rIndex), robots_(solver.robots())
{
  type_ = "torque";
  name_ = std::string("torque_") + robots_.robot(rIndex_).name();
  torque(robots_.robot(rIndex_).mbc().jointTorque);
  reset();
}

void TorqueTask::torque(const std::vector<std::vector<double>> & tau)
{
  if(tau.size() != robots_.robot(rIndex_).jointTorque().size())
  {
    mc_rtc::log::error_and_throw("[{}] Input torque vector has size {}, expected {}", name(), tau.size(),
                                 robots_.robot(rIndex_).jointTorque().size());
  }

  torque_ = tau;
}

void TorqueTask::torque(const std::string & jointName, std::vector<double> tau)
{
  if(!robots_.robot(rIndex_).hasJoint(jointName))
  {
    mc_rtc::log::error_and_throw("[{}] No joint named {} in {}", name(), jointName, robots_.robot(rIndex_).name());
  }

  auto jIndex = static_cast<int>(robots_.robot(rIndex_).jointIndexByName(jointName));
  torque_[jIndex] = tau;
}

std::vector<std::vector<double>> TorqueTask::torque() const
{
  return torque_;
}

void TorqueTask::target(const std::map<std::string, std::vector<double>> & joints)
{
  auto tau = torque_;
  Eigen::VectorXd dimW = dimWeight();

  const auto & mb = robots_.robot(rIndex_).mb();

  for(const auto & j : joints)
  {
    if(!robots_.robot(rIndex_).hasJoint(j.first))
    {
      mc_rtc::log::error_and_throw("[{}] No joint named {} in {}", name(), j.first, robots_.robot(rIndex_).name());
    }
    auto jIndex = static_cast<int>(robots_.robot(rIndex_).jointIndexByName(j.first));
    tau[jIndex] = j.second;

    // add a dimweight by default for every used joint
    if(mb.joint(jIndex).dof() > 0 && dimW[mb.jointPosInDof(jIndex)] == 0) { dimW[mb.jointPosInDof(jIndex)] = 1; }
  }

  torque(tau);
  dimWeight(dimW);
}

void TorqueTask::reset()
{
  // torque(robots_.robot(rIndex_).mbc().jointTorque);
  for(size_t i = 0; i < torque_.size(); i++)
  {
    if(torque_.size() != 0) { torque_[i] = {0}; }
  }
  Eigen::VectorXd dimW = dimWeight();
  dimW.setZero();
  dimWeight(dimW);
}

Eigen::VectorXd TorqueTask::jointsToDofs(const std::vector<std::vector<double>> & joints)
{
  const auto & mb = robots_.robot(rIndex_).mb();

  Eigen::VectorXd result = Eigen::VectorXd::Zero(mb.nrDof());

  for(size_t i = 0; i < joints.size(); ++i)
  {
    auto jIndex = static_cast<int>(i);
    const auto & joint = mb.joint(jIndex);
    const auto & joint_torque = joints[jIndex];

    size_t dofIndex = mb.jointPosInDof(jIndex);

    for(size_t j = 0; static_cast<int>(j) < joint.dof(); ++j)
    {
      if(j < joint_torque.size()) { result(dofIndex + j) = joint_torque[j]; }
      else { result(dofIndex + j) = 0.0; }
    }
  }
  return result;
}

void TorqueTask::update(mc_solver::QPSolver & solver)
{
  auto & robot = robots_.robot(rIndex_);
  rbd::ForwardDynamics fd(robot.mb());
  fd.computeH(robot.mb(), robot.mbc());
  fd.computeC(robot.mb(), robot.mbc());
  const Eigen::MatrixXd & M = fd.H();
  const Eigen::VectorXd & Cg = fd.C();
  const Eigen::VectorXd tau_d = jointsToDofs(torque_);
  Eigen::VectorXd refAccel = M.ldlt().solve(tau_d - Cg);
  PostureTask::refAccel(refAccel);
  PostureTask::update(solver);
}

void TorqueTask::addToLogger(mc_rtc::Logger & logger)
{
  PostureTask::addToLogger(logger);
  logger.addLogEntry(name_ + "_tau_d ", this,
                     [this]()
                     {
                       std::vector<double> tau(robots_.robot(rIndex_).refJointOrder().size(), 0);
                       auto & robot = robots_.robot(rIndex_);
                       for(size_t i = 0; i < tau.size(); i++)
                       {
                         auto mbcIndex = robot.jointIndexInMBC(i);
                         if(mbcIndex != -1) { tau[i] = torque_[static_cast<size_t>(mbcIndex)][0]; }
                       }
                       return tau;
                     });
}

void TorqueTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  MetaTask::addToGUI(gui);
  PostureTask::addToGUI(gui);

  for(const auto & j : robots_.robot(rIndex_).mb().joints())
  {
    auto jIndex = robots_.robot(rIndex_).jointIndexByName(j.name());
    const auto & mb = robots_.robot(rIndex_).mb();
    auto updateTorque = [this, mb](unsigned int jIndex, double tau)
    {
      if(tau > robots_.robot(rIndex_).tu()[jIndex][0]) tau = robots_.robot(rIndex_).tu()[jIndex][0];
      if(tau < robots_.robot(rIndex_).tl()[jIndex][0]) tau = robots_.robot(rIndex_).tl()[jIndex][0];

      Eigen::VectorXd dimW = dimWeight();
      if(mb.joint(jIndex).dof() > 0 && dimW[mb.jointPosInDof(jIndex)] == 0) { dimW[mb.jointPosInDof(jIndex)] = 1; }
      dimWeight(dimW);

      this->torque_[jIndex][0] = tau;
      torque(torque_);
    };
    auto updateDimWeight = [this, mb](unsigned int jIndex, double weight)
    {
      Eigen::VectorXd dimW = dimWeight();
      if(mb.joint(jIndex).dof() > 0) { dimW[mb.jointPosInDof(jIndex)] = weight; }
      dimWeight(dimW);
    };
    if(robots_.robot(rIndex_).tl()[jIndex].size() != 0)
    {
      gui.addElement({"Tasks", name_, "Target"},
                     mc_rtc::gui::NumberInput(
                         j.name(), [this, jIndex]() { return this->torque_[jIndex][0]; },
                         [jIndex, updateTorque](double tau) { updateTorque(jIndex, tau); }));
    }
    if(dimWeight().size())
    {
      gui.addElement({"Tasks", name_, "Gains", "Dimensional weights"},
                     mc_rtc::gui::NumberInput(
                         j.name(), [this, jIndex, mb]() { return this->dimWeight()[mb.jointPosInDof(jIndex)]; },
                         [jIndex, updateDimWeight, mb](double weight) { updateDimWeight(jIndex, weight); }));
    }
  }
}

} // namespace mc_tasks

namespace
{
static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "torque",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      const auto robotIndex = robotIndexFromConfig(config, solver.robots(), "torque");
      auto t = std::make_shared<mc_tasks::TorqueTask>(solver, robotIndex, config("weight", 10.));
      if(config.has("tau_d")) { t->torque(config("tau_d")); }
      t->load(solver, config);
      return t;
    });
} // namespace
