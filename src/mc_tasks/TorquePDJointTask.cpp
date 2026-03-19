#include <mc_tasks/TorquePDJointTask.h>

#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/ArrayLabel.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/NumberSlider.h>

namespace mc_tasks
{

TorquePDJointTask::TorquePDJointTask(const mc_solver::QPSolver & solver,
                                     unsigned int rIndex,
                                     double stiffness,
                                     double weight)
: TorqueTask(solver, rIndex, weight), robots_(solver.robots()), rIndex_(rIndex),
  nbActuatedJoints(
      (robots_.robot(rIndex_).mb().nrJoints() > 0 && robots_.robot(rIndex_).mb().joint(0).type() == rbd::Joint::Free)
          ? robots_.robot(rIndex_).mb().nrDof() - 6
          : robots_.robot(rIndex_).mb().nrDof()),
  stiffness_(Eigen::VectorXd::Zero(nbActuatedJoints)), damping_(Eigen::VectorXd::Zero(nbActuatedJoints)),
  posTarget_(Eigen::VectorXd::Zero(nbActuatedJoints)), velTarget_(Eigen::VectorXd::Zero(nbActuatedJoints)),
  torqueFeedforward_(Eigen::VectorXd::Zero(nbActuatedJoints)), posError_(Eigen::VectorXd::Zero(nbActuatedJoints)),
  velError_(Eigen::VectorXd::Zero(nbActuatedJoints)), torqueTarget_(Eigen::VectorXd::Zero(nbActuatedJoints))
{
  if(backend_ == Backend::Tasks)
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_tasks] Can't use TorquePDJointTask with {} backend, please use TVM backend", backend_);
  name_ = std::string("pd_joint_") + solver.robots().robot(rIndex_).name();
  type_ = "pd_joint";

  setStiffness(stiffness);
  setDamping(2.0 * sqrt(stiffness)); // Critical damping by default
  reset();
}

void TorquePDJointTask::reset()
{
  posTarget_ = rbd::sParamToVector(robots_.robot(rIndex_).mb(), robots_.robot(rIndex_).q()).tail(nbActuatedJoints);
  velTarget_.setZero();
  torqueFeedforward_.setZero();
}

void TorquePDJointTask::update(mc_solver::QPSolver & solver)
{
  torqueTarget_.setZero();
  auto & realRobot = solver.realRobots().robot(rIndex_);
  Eigen::VectorXd qdot_full(realRobot.mb().nrDof()), q_full(realRobot.mb().nrParams());
  qdot_full = rbd::sDofToVector(realRobot.mb(), realRobot.alpha());
  q_full = rbd::sParamToVector(realRobot.mb(), realRobot.q());

  posError_ = posTarget_ - q_full.tail(nbActuatedJoints);
  velError_ = velTarget_ - qdot_full.tail(nbActuatedJoints);
  torqueTarget_ += stiffness_.asDiagonal() * posError_;
  torqueTarget_ += damping_.asDiagonal() * velError_;
  torqueTarget_ += torqueFeedforward_;

  std::vector<std::vector<double>> torque_vector = robots_.robot(rIndex_).mbc().jointTorque;

  Eigen::VectorXd torque_target_full = Eigen::VectorXd::Zero(realRobot.mb().nrDof());
  torque_target_full.tail(nbActuatedJoints) = torqueTarget_;

  torque_vector = rbd::sVectorToDof(realRobot.mb(), torque_target_full);

  TorqueTask::torque(torque_vector);
  TorqueTask::update(solver);
}

void TorquePDJointTask::setStiffness(double stiffness)
{
  stiffness_ = Eigen::VectorXd::Constant(nbActuatedJoints, stiffness);
}

void TorquePDJointTask::setDamping(double damping)
{
  damping_ = Eigen::VectorXd::Constant(nbActuatedJoints, damping);
}

void TorquePDJointTask::setStiffness(const Eigen::VectorXd & stiffness)
{
  if(stiffness.size() != nbActuatedJoints)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[TorquePDJointTask] Stiffness vector size should be {}, got {}",
                                                     nbActuatedJoints, stiffness.size());
  }
  stiffness_ = stiffness;
}

void TorquePDJointTask::setDamping(const Eigen::VectorXd & damping)
{
  if(damping.size() != nbActuatedJoints)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[TorquePDJointTask] Damping vector size should be {}, got {}",
                                                     nbActuatedJoints, damping.size());
  }
  damping_ = damping;
}

void TorquePDJointTask::setPosTarget(const Eigen::VectorXd & qd)
{
  if(qd.size() != nbActuatedJoints)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[TorquePDJointTask] Position target vector size should be {}, got {}", nbActuatedJoints, qd.size());
  }
  posTarget_ = qd;
}

void TorquePDJointTask::setVelTarget(const Eigen::VectorXd & qd_dot)
{
  if(qd_dot.size() != nbActuatedJoints)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[TorquePDJointTask] Velocity target vector size should be {}, got {}", nbActuatedJoints, qd_dot.size());
  }
  velTarget_ = qd_dot;
}

void TorquePDJointTask::setTorqueFeedforward(const Eigen::VectorXd & tau_ff)
{
  if(tau_ff.size() != nbActuatedJoints)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[TorquePDJointTask] Torque feedforward vector size should be {}, got {}", nbActuatedJoints, tau_ff.size());
  }
  torqueFeedforward_ = tau_ff;
}

const Eigen::VectorXd & TorquePDJointTask::stiffness() const
{
  return stiffness_;
}

const Eigen::VectorXd & TorquePDJointTask::damping() const
{
  return damping_;
}

const Eigen::VectorXd & TorquePDJointTask::posTarget() const
{
  return posTarget_;
}

const Eigen::VectorXd & TorquePDJointTask::velTarget() const
{
  return velTarget_;
}

const Eigen::VectorXd & TorquePDJointTask::torqueFeedforward() const
{
  return torqueFeedforward_;
}

void TorquePDJointTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement(
      {"Tasks", name_, "Gains"}, mc_rtc::gui::ArrayInput("Stiffness", stiffness_),
      mc_rtc::gui::ArrayInput("Damping", damping_),
      mc_rtc::gui::NumberInput(
          "Constant Stiffness & Critical Damping", [this]() { return stiffness_[0]; },
          [this](const double & g)
          {
            setStiffness(g);
            setDamping(2.0 * sqrt(g));
          }),
      mc_rtc::gui::NumberInput(
          "Constant Stiffness", [this]() { return stiffness_[0]; }, [this](const double & s) { setStiffness(s); }),
      mc_rtc::gui::NumberInput(
          "Constant Damping", [this]() { return damping_[0]; }, [this](const double & d) { setDamping(d); }));

  gui.addElement({"Tasks", name_, "Details"}, mc_rtc::gui::ArrayLabel("Position Error", posError_),
                 mc_rtc::gui::ArrayLabel("Velocity Error", velError_));

  std::vector<std::string> active_gripper_joints;
  const auto & robot = robots_.robot(rIndex_);

  for(const auto & g : robot.grippers())
  {
    for(const auto & n : g.get().activeJoints()) { active_gripper_joints.push_back(n); }
  }
  auto isActiveGripperJoint = [&](const std::string & j)
  { return std::find(active_gripper_joints.begin(), active_gripper_joints.end(), j) != active_gripper_joints.end(); };

  int i = 0;
  for(const auto & j : robot.mb().joints())
  {
    if(j.dof() != 1 || j.isMimic() || isActiveGripperJoint(j.name())) { continue; }
    auto jIndex = robot.jointIndexByName(j.name());
    bool isContinuous = robot.ql()[jIndex][0] == -std::numeric_limits<double>::infinity();
    auto updatePosTarget = [this](int i, double v)
    {
      this->posTarget_[i] = v;
      setPosTarget(posTarget_);
    };

    auto updateVelTarget = [this](int i, double v)
    {
      this->velTarget_[i] = v;
      setVelTarget(velTarget_);
    };

    auto updateTorqueFeedforward = [this](int i, double v)
    {
      this->torqueFeedforward_[i] = v;
      setTorqueFeedforward(torqueFeedforward_);
    };

    if(isContinuous)
    {
      gui.addElement({"Tasks", name_, "Position Target"},
                     mc_rtc::gui::NumberInput(
                         j.name(), [this, i]() { return this->posTarget_[i]; },
                         [i, updatePosTarget](double v) { updatePosTarget(i, v); }));
    }
    else
    {
      gui.addElement({"Tasks", name_, "Position Target"},
                     mc_rtc::gui::NumberSlider(
                         j.name(), [this, i]() { return this->posTarget_[i]; }, [i, updatePosTarget](double v)
                         { updatePosTarget(i, v); }, robot.ql()[jIndex][0], robot.qu()[jIndex][0]));
    }

    gui.addElement({"Tasks", name_, "Velocity Target"},
                   mc_rtc::gui::NumberSlider(
                       j.name(), [this, i]() { return this->velTarget_[i]; }, [i, updateVelTarget](double v)
                       { updateVelTarget(i, v); }, robot.vl()[jIndex][0], robot.vu()[jIndex][0]));

    gui.addElement({"Tasks", name_, "Torque Feedforward"},
                   mc_rtc::gui::NumberSlider(
                       j.name(), [this, i]() { return this->torqueFeedforward_[i]; },
                       [i, updateTorqueFeedforward](double v) { updateTorqueFeedforward(i, v); },
                       -robot.tl()[jIndex][0], robot.tu()[jIndex][0]));
    i++;
  }
  TorqueTask::addToGUI(gui);
}

void TorquePDJointTask::addToLogger(mc_rtc::Logger & logger)
{
  TorqueTask::addToLogger(logger);
  logger.removeLogEntry(name_ + "_torque");
  logger.addLogEntry(name_ + "_stiffness", [this]() { return stiffness_; });
  logger.addLogEntry(name_ + "_damping", [this]() { return damping_; });
  logger.addLogEntry(name_ + "_posTarget", [this]() { return posTarget_; });
  logger.addLogEntry(name_ + "_velTarget", [this]() { return velTarget_; });
  logger.addLogEntry(name_ + "_torqueFeedforward", [this]() { return torqueFeedforward_; });
  logger.addLogEntry(name_ + "_posError", [this]() { return posError_; });
  logger.addLogEntry(name_ + "_velError", [this]() { return velError_; });
  logger.addLogEntry(name_ + "_torque", [this]() { return torqueTarget_; });
}

} // namespace mc_tasks
