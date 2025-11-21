/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/WrenchTask.h>

#include <mc_tvm/WrenchFunction.h>

#include <mc_tasks/MetaTaskLoader.h>

// #include <mc_solver/TasksQPSolver.h>

#include <mc_rbdyn/rpy_utils.h>

#include <mc_rbdyn/hat.h>
#include <mc_rtc/ConfigurationHelpers.h>
#include <mc_rtc/deprecated.h>
#include <mc_rtc/gui/Force.h>
#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_tasks
{

static inline mc_rtc::void_ptr_caster<mc_tvm::WrenchFunction> tvm_error{};

WrenchTask::WrenchTask(const mc_rbdyn::RobotFrame & frame, double weight)
: TrajectoryTaskGeneric(frame.robot().robots(), frame.robot().robotIndex(), 0, weight), frame_(frame)
{
  finalize<Backend::TVM, mc_tvm::WrenchFunction>(frame);
  type_ = "wrench";
  name_ = "wrench_" + frame.robot().name() + "_" + frame.name();
  isNoneTaskDynamics_ = true;
}

WrenchTask::WrenchTask(const std::string & surfaceName,
                       const mc_rbdyn::Robots & robots,
                       unsigned int robotIndex,
                       double weight)
: WrenchTask(robots.robot(robotIndex).frame(surfaceName), weight)
{
}

void WrenchTask::reset()
{
  TrajectoryTaskGeneric::reset();
  switch(backend_)
  {
    // case Backend::Tasks:
    //   tasks_error(errorT)->target(frame_->position());
    //   break;
    case Backend::TVM:
      tvm_error(errorT)->reset();
      break;
    default:
      break;
  }
}

sva::ForceVecd WrenchTask::target() const
{
  switch(backend_)
  {
    // case Backend::Tasks:
    //   return tasks_error(errorT)->target();
    case Backend::TVM:
      return tvm_error(errorT)->wrench();
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

void WrenchTask::target(const sva::ForceVecd & worldWrench)
{
  switch(backend_)
  {
    // case Backend::Tasks:
    //   tasks_error(errorT)->target(worldwrench);
    //   break;
    case Backend::TVM:
      tvm_error(errorT)->wrench(worldWrench);
      break;
    default:
      mc_rtc::log::error("Not implemented");
      break;
  }
}

sva::ForceVecd WrenchTask::surfaceWrench() const noexcept
{
  switch(backend_)
  {
    case Backend::TVM:
      return tvm_error(errorT)->wrench();
    default:
      mc_rtc::log::error("Not implemented");
      return sva::ForceVecd::Zero();
  }
}

void WrenchTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  logger.addLogEntry(name_ + "_wrench", this, [this]() { return surfaceWrench(); });
  logger.addLogEntry(name_ + "_target_wrench", this, [this]() { return target(); });
}

void WrenchTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTaskGeneric::addToGUI(gui);
  auto fConf_wrench_target = mc_rtc::gui::ForceConfig();
  fConf_wrench_target.color = mc_rtc::gui::Color::Blue;
  fConf_wrench_target.force_scale = 0.01;

  auto fConf_wrench = mc_rtc::gui::ForceConfig();
  fConf_wrench.color = mc_rtc::gui::Color::Yellow;
  fConf_wrench.force_scale = 0.01;
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Force(
                     "wrench_target", fConf_wrench_target, [this]() { return this->target(); },
                     [this]() { return this->frame_->position(); }),
                 mc_rtc::gui::Force(
                     "wrench", fConf_wrench, [this]() { return this->surfaceWrench(); },
                     [this]() { return this->frame_->position(); }));
}

} // namespace mc_tasks
