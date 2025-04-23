#include <mc_tasks/TorqueTask.h>

#include <mc_tasks/MetaTaskLoader.h>
#include <mc_solver/TasksQPSolver.h>

#include <mc_rtc/gui/NumberInput.h>

#include <Tasks/QPTasks.h>
#include <Tasks/Bounds.h>
#include "mc_rtc/logging.h"
#include <string>

namespace mc_tasks
{
static inline mc_rtc::void_ptr_caster<tasks::qp::TorqueTask> tasks_torque{};

TorqueTask::TorqueTask(const mc_rbdyn::Robots & robots, unsigned int rIndex, double weight)
: robots_(robots), rIndex_(rIndex), pt_(nullptr, [](void*){})
{
  type_ = "torque";
  name_ = "torque_" + robots.robot(rIndex).name();
  
  // Initialize target torques to zero
  targetTorques_ = Eigen::VectorXd::Zero(robots_.robot(rIndex_).mb().nrDof());
  
  // Create torque bounds
  std::vector<std::vector<double>> lTB(robots_.robot(rIndex_).mb().nrDof(), std::vector<double>(1, -INFINITY));
  std::vector<std::vector<double>> uTB(robots_.robot(rIndex_).mb().nrDof(), std::vector<double>(1, INFINITY));
  tasks::TorqueBound tb(lTB, uTB);
  
  Eigen::VectorXd joints = Eigen::VectorXd::Ones(robots_.robot(rIndex_).mb().nrDof());
  pt_ = mc_rtc::make_void_ptr<tasks::qp::TorqueTask>(robots_.mbs(), static_cast<int>(rIndex_), tb, joints, weight);
  
  this->weight(weight);
}

void TorqueTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  MetaTask::load(solver, config);
  if(config.has("torques")) {this->torques(config("torques")); }
  if(config.has("weight")) { this->weight(config("weight")); }
}

void TorqueTask::reset()
{
  targetTorques_.setZero();
  torques(targetTorques_);
}

const Eigen::VectorXd & TorqueTask::torques() const
{
  return targetTorques_;
}

void TorqueTask::torques(const Eigen::VectorXd & t)
{
  if (t.size() != robots_.robot(rIndex_).mb().nrDof())
  {
    mc_rtc::log::error("[{}] Input torque vector has size {}, expected {}", 
                       name(), t.size(), robots_.robot(rIndex_).mb().nrDof());
    return;
  }
  
  targetTorques_ = t;
  
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_torque(pt_)->update(robots_.mbs(), robots_.mbcs(), {});
      break;
    case Backend::TVM:
      // TVM backend not implemented yet
      break;
    default:
      break;
  }
}

void TorqueTask::torque(const std::string & jointName, double t)
{
  if(!robots_.robot(rIndex_).hasJoint(jointName))
  {
    mc_rtc::log::error_and_throw("[{}] No joint named {} in {}", name(), jointName, robots_.robot(rIndex_).name());
  }
  auto jIndex = static_cast<int>(robots_.robot(rIndex_).jointIndexByName(jointName));
  const auto & joint = robots_.robot(rIndex_).mb().joint(jIndex);
  auto dofIndex = robots_.robot(rIndex_).mb().jointPosInDof(jIndex);
  
  for(int i = 0; i < joint.dof(); ++i)
  {
    targetTorques_[dofIndex + i] = t;
  }
  
  torques(targetTorques_);
}

void TorqueTask::torque(const std::string & jointName, const Eigen::VectorXd & t)
{
  if(!robots_.robot(rIndex_).hasJoint(jointName))
  {
    mc_rtc::log::error_and_throw("[{}] No joint named {} in {}", name(), jointName, robots_.robot(rIndex_).name());
  }
  auto jIndex = static_cast<int>(robots_.robot(rIndex_).jointIndexByName(jointName));
  const auto & joint = robots_.robot(rIndex_).mb().joint(jIndex);
  auto dofIndex = robots_.robot(rIndex_).mb().jointPosInDof(jIndex);
  
  if(t.size() != joint.dof())
  {
    mc_rtc::log::error_and_throw("[{}] Joint {} has {} DOFs, but {} torques were provided", 
                                name(), jointName, joint.dof(), t.size());
  }
  
  for(int i = 0; i < joint.dof(); ++i)
  {
    targetTorques_[dofIndex + i] = t[i];
  }
  
  torques(targetTorques_);
}

Eigen::VectorXd TorqueTask::currentTorques() const
{
  const auto & jointTorque = robot().mbc().jointTorque;
  const auto & mb = robot().mb();
  
  Eigen::VectorXd result = Eigen::VectorXd::Zero(mb.nrDof());
  
  for(size_t i = 0; i < jointTorque.size(); ++i)
  {
    const auto & joint = mb.joint(i);
    const auto & joint_torques = jointTorque[i];
    
    size_t dofIndex = mb.jointPosInDof(i);
    
    for(size_t j = 0; j < joint.dof(); ++j)
    {
      if (j < joint_torques.size()) {
        result(dofIndex + j) = joint_torques[j];
      } else {
        result(dofIndex + j) = 0.0;
      }
    }
  }
  return result;
}

Eigen::VectorXd TorqueTask::eval() const
{
  switch(backend_)
  {
    case Backend::Tasks:
      return tasks_torque(pt_)->C();
    case Backend::TVM:
      // TVM backend not implemented yet
      mc_rtc::log::error_and_throw("TVM backend not implemented for TorqueTask");
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

Eigen::VectorXd TorqueTask::speed() const
{
  switch(backend_)
  {
    case Backend::Tasks:
      return tasks_torque(pt_)->jointSelect();
    case Backend::TVM:
      // TVM backend not implemented yet
      mc_rtc::log::error_and_throw("TVM backend not implemented for TorqueTask");
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

void TorqueTask::addToSolver(mc_solver::QPSolver & solver)
{
  if(!inSolver_)
  {
    tasks_solver(solver).addTask(tasks_torque(pt_));
    inSolver_ = true;
  }
}

void TorqueTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  if(inSolver_)
  {
    tasks_solver(solver).removeTask(tasks_torque(pt_));
    inSolver_ = false;
  }
}

void TorqueTask::update(mc_solver::QPSolver & solver)
{
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_torque(pt_)->update(robots_.mbs(), robots_.mbcs(), {});
      break;
    case Backend::TVM:
      // TVM backend not implemented yet
      break;
    case Backend::Unset:
      break;
    default:
      break;
  }
}

void TorqueTask::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(name_ + "_eval", this, [this]() { return eval(); });
  logger.addLogEntry(name_ + "_speed", this, [this]() { return speed(); });
  logger.addLogEntry(name_ + "_torques", this, [this]() { return torques(); });
}

void TorqueTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  MetaTask::addToGUI(gui);
  gui.addElement({"Tasks", name_, "Gains"},
                 mc_rtc::gui::NumberInput(
                     "weight", [this]() { return this->weight(); }, [this](const double & w) { this->weight(w); }));
}

void TorqueTask::selectActiveJoints(mc_solver::QPSolver & solver,
                                    const std::vector<std::string> & activeJointsName,
                                    const std::map<std::string, std::vector<std::array<int, 2>>> &)
{
  ensureHasJoints(robots_.robot(rIndex_), activeJointsName, "[" + name() + "::selectActiveJoints]");
  std::vector<std::string> inactiveJoints = {};
  for(const auto & j : robots_.robot(rIndex_).mb().joints())
  {
    if(j.dof() && std::find(activeJointsName.begin(), activeJointsName.end(), j.name()) == activeJointsName.end())
    {
      inactiveJoints.push_back(j.name());
    }
  }
  selectInactiveJoints(solver, inactiveJoints);
}

void TorqueTask::selectUnactiveJoints(mc_solver::QPSolver & solver,
                                      const std::vector<std::string> & inactiveJointsName,
                                      const std::map<std::string, std::vector<std::array<int, 2>>> & vect)
{
  mc_rtc::log::warning("selectUnactiveJoints: should be selectInactiveJoints instead. Still working but should not be called.");
  selectInactiveJoints(solver, inactiveJointsName, vect);
}

void TorqueTask::selectInactiveJoints(mc_solver::QPSolver &,
                                      const std::vector<std::string> & inactiveJointsName,
                                      const std::map<std::string, std::vector<std::array<int, 2>>> &)
{
  ensureHasJoints(robots_.robot(rIndex_), inactiveJointsName, "[" + name() + "::selectInActiveJoints]");
  Eigen::VectorXd dimW = dimWeight();
  dimW.setOnes();
  const auto & robot = robots_.robot(rIndex_);
  auto dofOffset = [&robot, this]()
  {
    switch(backend_)
    {
      case Backend::Tasks:
        return 0;
      case Backend::TVM:
        return robot.mb().joint(0).dof();
      default:
        mc_rtc::log::error_and_throw("Not implemented in backend {}", backend_);
    }
  }();
  for(const auto & j : inactiveJointsName)
  {
    auto jIndex = static_cast<int>(robot.jointIndexByName(j));
    const auto & joint = robot.mb().joint(jIndex);
    if(joint.dof() == 6) { continue; }
    auto dofIndex = robot.mb().jointPosInDof(jIndex) - dofOffset;
    dimW.segment(dofIndex, joint.dof()).setZero();
  }
  dimWeight(dimW);
}

void TorqueTask::resetJointsSelector(mc_solver::QPSolver & solver)
{
  selectInactiveJoints(solver, {});
}

void TorqueTask::dimWeight(const Eigen::VectorXd & dimW)
{
  auto* currentTask = tasks_torque(pt_);
  std::vector<std::vector<double>> lTB(robots_.robot(rIndex_).mb().nrDof(), std::vector<double>(1, -INFINITY));
  std::vector<std::vector<double>> uTB(robots_.robot(rIndex_).mb().nrDof(), std::vector<double>(1, INFINITY));
  tasks::TorqueBound tb(lTB, uTB);
  
  switch(backend_)
  {
    case Backend::Tasks:
      // Create a new task with the updated joint selector
      pt_ = mc_rtc::make_void_ptr<tasks::qp::TorqueTask>(robots_.mbs(), static_cast<int>(rIndex_), tb, dimW, currentTask->weight());
      break;
    case Backend::TVM:
      // TVM backend not implemented yet
      break;
    case Backend::Unset:
      break;
    default:
      break;
  }
}

Eigen::VectorXd TorqueTask::dimWeight() const
{
  switch(backend_)
  {
    case Backend::Tasks:
      return tasks_torque(pt_)->jointSelect();
    case Backend::TVM:
      // TVM backend not implemented yet
      mc_rtc::log::error_and_throw("TVM backend not implemented for TorqueTask");
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

void TorqueTask::weight(double w)
{
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_torque(pt_)->weight(w);
      break;
    case Backend::TVM:
      // TVM backend not implemented yet
      break;
    default:
      break;
  }
}

double TorqueTask::weight() const
{
  switch(backend_)
  {
    case Backend::Tasks:
      return tasks_torque(pt_)->weight();
    case Backend::TVM:
      // TVM backend not implemented yet
      mc_rtc::log::error_and_throw("TVM backend not implemented for TorqueTask");
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

} //namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "torque",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      auto robotIndex = config("robotIndex", 0);
      auto weight = config("weight", 1000.0);
      auto task = std::make_shared<mc_tasks::TorqueTask>(solver.robots(), robotIndex, weight);
      task->load(solver, config);
      return task;
    });

} // namespace
