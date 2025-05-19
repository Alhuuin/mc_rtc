#include <mc_rbdyn/Robot.h>
#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/ArrayLabel.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/log/Logger.h>
#include <mc_solver/QPSolver.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/TorqueTask.h>
#include <RBDyn/FD.h>
#include "mc_rtc/logging.h"

namespace mc_tasks
{

TorqueTask::TorqueTask(const mc_solver::QPSolver & solver, unsigned int rIndex, double weight)
: PostureTask(solver, rIndex, 0.1, weight), dt_(solver.dt()), rIndex_(rIndex), robots_(solver.robots())
{
  torques_ = robots_.robot(rIndex_).mbc().jointTorque;
  type_ = "torque";
  name_ = std::string("torque_") + robots_.robot(rIndex_).name();
}

void TorqueTask::torques(const std::vector<std::vector<double>> & tau)
{
  if(tau.size() != robots_.robot(rIndex_).jointTorque().size())
  {
    mc_rtc::log::error_and_throw("[{}] Input torque vector has size {}, expected {}", name(), tau.size(),
                                 robots_.robot(rIndex_).jointTorque().size());
  }

  torques_ = tau;
}

void TorqueTask::torques(const std::string & jointName, std::vector<double> tau)
{
  if(!robots_.robot(rIndex_).hasJoint(jointName))
  {
    mc_rtc::log::error_and_throw("[{}] No joint named {} in {}", name(), jointName, robots_.robot(rIndex_).name());
  }

  auto jIndex = static_cast<int>(robots_.robot(rIndex_).jointIndexByName(jointName));
  torques_[jIndex] = tau;
}

std::vector<std::vector<double>> TorqueTask::torques() const
{
  return torques_;
}

void TorqueTask::target(const std::map<std::string, std::vector<double>> & joints)
{
  for(const auto & j : joints) { torques(j.first, j.second); }
}

// will be removed
Eigen::VectorXd TorqueTask::currentTorques() const
{
  const auto & jointTorque = robots_.robot(rIndex_).mbc().jointTorque;
  const auto & mb = robots_.robot(rIndex_).mb();

  Eigen::VectorXd result = Eigen::VectorXd::Zero(mb.nrDof());

  for(size_t i = 0; i < jointTorque.size(); ++i)
  {
    auto jIndex = static_cast<int>(i);
    const auto & joint = mb.joint(jIndex);
    const auto & joint_torques = jointTorque[jIndex];

    size_t dofIndex = mb.jointPosInDof(jIndex);

    for(size_t j = 0; j < joint.dof(); ++j)
    {
      if(j < joint_torques.size()) { result(dofIndex + j) = joint_torques[j]; }
      else { result(dofIndex + j) = 0.0; }
    }
  }
  return result;
}

void TorqueTask::reset()
{
  PostureTask::reset();
}

Eigen::VectorXd TorqueTask::jointsToDofs(const std::vector<std::vector<double>> & joints)
{
  const auto & mb = robots_.robot(rIndex_).mb();

  Eigen::VectorXd result = Eigen::VectorXd::Zero(mb.nrDof());

  for(size_t i = 0; i < joints.size(); ++i)
  {
    auto jIndex = static_cast<int>(i);
    const auto & joint = mb.joint(jIndex);
    const auto & joint_torques = joints[jIndex];

    size_t dofIndex = mb.jointPosInDof(jIndex);

    for(size_t j = 0; j < joint.dof(); ++j)
    {
      if(j < joint_torques.size()) { result(dofIndex + j) = joint_torques[j]; }
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
  const Eigen::VectorXd tau_d = jointsToDofs(torques_);
  Eigen::VectorXd refAccel = M.ldlt().solve(tau_d - Cg);
  PostureTask::refAccel(refAccel);
  PostureTask::update(solver);
}

void TorqueTask::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(name_ + "_tau_d ", this, [this]() { return jointsToDofs(torques_); });
}

void TorqueTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  MetaTask::addToGUI(gui);
  gui.addElement({"Tasks", name_, "Torques"},
                 mc_rtc::gui::ArrayInput(
                     "Desired Torques", [this]() { return this->jointsToDofs(this->torques_); },
                     [this](const std::vector<std::vector<double>> & tau) { this->torques(tau); }));
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
      if(config.has("tau_d")) { t->torques(config("tau_d")); }
      // if(config.has("desiredTorque")) { t->desiredTorque(config("desiredTorque")); }
      t->load(solver, config);
      return t;
    });
} // namespace

// namespace mc_tasks
// {

// TorqueTask::TorqueTask(const mc_solver::QPSolver & solver, unsigned int rIndex, double weight)
// : PostureTask(solver, rIndex, 0.1, weight), dt_(solver.dt()), rIndex_(rIndex), robots_(solver.robots())
// {
//   desiredTorque_ = Eigen::VectorXd::Zero(robots_.robot(rIndex_).mb().nrDof());
//   type_ = "torque";
//   name_ = std::string("torque_") + robots_.robot(rIndex_).name();
// }

// void TorqueTask::desiredTorque(const Eigen::VectorXd & tau)
// {
//   if(tau.size() != robots_.robot(rIndex_).mb().nrDof())
//   {
//     mc_rtc::log::error("[{}] Input torque vector has size {}, expected {}", name(), tau.size(),
//                        robots_.robot(rIndex_).mb().nrDof());
//     return;
//   }

//   desiredTorque_ = tau;
// }

// void TorqueTask::desiredTorque(const std::string & jointName, double tau)
// {
//   if(!robots_.robot(rIndex_).hasJoint(jointName))
//   {
//     mc_rtc::log::error_and_throw("[{}] No joint named {} in {}", name(), jointName, robots_.robot(rIndex_).name());
//   }

//   auto jIndex = static_cast<int>(robots_.robot(rIndex_).jointIndexByName(jointName));
//   const auto & joint = robots_.robot(rIndex_).mb().joint(jIndex);
//   auto dofIndex = robots_.robot(rIndex_).mb().jointPosInDof(jIndex);

//   desiredTorque_(dofIndex) = tau;
// }

// const Eigen::VectorXd & TorqueTask::desiredTorque() const
// {
//   return desiredTorque_;
// }

// Eigen::VectorXd TorqueTask::currentTorques() const
// {
//   const auto & jointTorque = robots_.robot(rIndex_).mbc().jointTorque;
//   const auto & mb = robots_.robot(rIndex_).mb();

//   Eigen::VectorXd result = Eigen::VectorXd::Zero(mb.nrDof());

//   for(size_t i = 0; i < jointTorque.size(); ++i)
//   {
//     const auto & joint = mb.joint(i);
//     const auto & joint_torques = jointTorque[i];

//     size_t dofIndex = mb.jointPosInDof(i);

//     for(size_t j = 0; j < joint.dof(); ++j)
//     {
//       if(j < joint_torques.size()) { result(dofIndex + j) = joint_torques[j]; }
//       else { result(dofIndex + j) = 0.0; }
//     }
//   }
//   return result;
// }

// void TorqueTask::reset()
// {
//   desiredTorque_.setZero();
//   PostureTask::reset();
// }

// void TorqueTask::update(mc_solver::QPSolver & solver)
// {
//   auto & robot = robots_.robot(rIndex_);
//   rbd::ForwardDynamics fd(robot.mb());
//   fd.computeH(robot.mb(), robot.mbc());
//   fd.computeC(robot.mb(), robot.mbc());
//   const Eigen::MatrixXd & M = fd.H();
//   const Eigen::VectorXd & Cg = fd.C();
//   Eigen::VectorXd refAccel = M.ldlt().solve(desiredTorque_ - Cg);
//   PostureTask::refAccel(refAccel);
//   PostureTask::update(solver);
// }

// void TorqueTask::addToLogger(mc_rtc::Logger & logger)
// {
//   logger.addLogEntry(name_ + "_tau_d ", this, [this]() { return desiredTorque_; });
// }

// void TorqueTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
// {
//   MetaTask::addToGUI(gui);
//   gui.addElement({"Tasks", name_, "Torques"},
//                  mc_rtc::gui::ArrayInput(
//                      "Desired Torques", [this]() { return this->desiredTorque_; },
//                      [this](const Eigen::VectorXd & tau) { this->desiredTorque(tau); }),
//                  mc_rtc::gui::ArrayLabel("Current Torques", [this] { return this->currentTorques(); }));
// }

// } // namespace mc_tasks

// namespace
// {
// static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
//     "torque",
//     [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
//     {
//       const auto robotIndex = robotIndexFromConfig(config, solver.robots(), "torque");
//       auto t = std::make_shared<mc_tasks::TorqueTask>(solver, robotIndex, config("weight", 10.));
//       if(config.has("desiredTorque")) { t->desiredTorque(config("desiredTorque")); }
//       t->load(solver, config);
//       return t;
//     });
// } // namespace