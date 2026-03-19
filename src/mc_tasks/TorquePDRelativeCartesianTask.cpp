#include <mc_tasks/TorquePDRelativeCartesianTask.h>

#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/ArrayLabel.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/NumberSlider.h>
#include <mc_rtc/gui/Transform.h>

namespace mc_tasks
{

TorquePDRelativeCartesianTask::TorquePDRelativeCartesianTask(const mc_solver::QPSolver & solver,
                                                             const mc_rbdyn::RobotFrame & frame,
                                                             const mc_rbdyn::RobotFrame & relative,
                                                             double stiffness,
                                                             double weight)
: TorqueTask(solver, frame.robot().robotIndex(), weight), robots_(solver.robots()), rIndex_(frame.robot().robotIndex()),
  nbActuatedJoints(
      (robots_.robot(rIndex_).mb().nrJoints() > 0 && robots_.robot(rIndex_).mb().joint(0).type() == rbd::Joint::Free)
          ? robots_.robot(rIndex_).mb().nrDof() - 6
          : robots_.robot(rIndex_).mb().nrDof()),
  stiffness_(Eigen::Vector6d::Zero()), damping_(Eigen::Vector6d::Zero()), posTarget_(sva::PTransformd::Identity()),
  velTarget_(sva::MotionVecd::Zero()), torqueFeedforward_(Eigen::VectorXd::Zero(nbActuatedJoints)),
  posError_(sva::MotionVecd::Zero()), velError_(sva::MotionVecd::Zero()),
  torqueTarget_(Eigen::VectorXd::Zero(nbActuatedJoints)), frame_(frame), relative_(relative)
{
  if(backend_ == Backend::Tasks)
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_tasks] Can't use TorquePDRelativeCartesianTask with {} backend, please use TVM backend", backend_);

  name_ = std::string("pd_relative_cartesian_") + solver.robots().robot(rIndex_).name();
  type_ = "pd_relative_cartesian";

  setStiffness(stiffness);
  setDamping(2.0 * sqrt(stiffness)); // Critical damping by default

  reset();
}

TorquePDRelativeCartesianTask::TorquePDRelativeCartesianTask(const mc_solver::QPSolver & solver,
                                                             const std::string & bodyName,
                                                             const std::string & relBodyName,
                                                             const mc_rbdyn::Robots & robots,
                                                             unsigned int rIndex,
                                                             double stiffness,
                                                             double weight)
: TorquePDRelativeCartesianTask(solver,
                                robots.robot(rIndex).frame(bodyName),
                                relBodyName.size() ? robots.robot(rIndex).frame(relBodyName)
                                                   : robots.robot(rIndex).frame(0),
                                stiffness,
                                weight)
{
}

TorquePDRelativeCartesianTask::TorquePDRelativeCartesianTask(const mc_solver::QPSolver & solver,
                                                             const std::string & bodyName,
                                                             const mc_rbdyn::Robots & robots,
                                                             unsigned int rIndex,
                                                             double stiffness,
                                                             double weight)
: TorquePDRelativeCartesianTask(solver,
                                robots.robot(rIndex).frame(bodyName),
                                robots.robot(rIndex).frame(0),
                                stiffness,
                                weight)
{
}

void TorquePDRelativeCartesianTask::reset()
{
  sva::PTransformd X_0_body = frame_->position();
  sva::PTransformd X_0_rel = relative_->position();
  posTarget_ = X_0_body * X_0_rel.inv(); // X_rel_body

  velTarget_.Zero();
  torqueFeedforward_.setZero();
}

void TorquePDRelativeCartesianTask::update(mc_solver::QPSolver & solver)
{

  sva::PTransformd X_0_body = frame_->position();
  sva::PTransformd X_0_rel = relative_->position();
  sva::PTransformd X_rel_body = X_0_body * (X_0_rel.inv());
  posError_ = sva::transformError(X_rel_body, posTarget_);

  sva::MotionVecd vel_0_body = frame_->velocity();
  sva::MotionVecd vel_0_rel = sva::MotionVecd::Zero();
  sva::MotionVecd vel_rel_body = vel_0_body - X_rel_body * vel_0_rel;
  velError_ = vel_rel_body - velTarget_;

  // Jacobians of both frames
  rbd::Jacobian jac_body(robots_.robot(rIndex_).mb(), frame_->body());
  rbd::Jacobian jac_rel(robots_.robot(rIndex_).mb(), relative_->body());

  // Body Jacobians (each in its own frame)
  Eigen::MatrixXd J_body = jac_body.bodyJacobian(robots_.robot(rIndex_).mb(), robots_.robot(rIndex_).mbc());
  Eigen::MatrixXd J_rel = jac_rel.bodyJacobian(robots_.robot(rIndex_).mb(), robots_.robot(rIndex_).mbc());

  // Express J_rel in body frame and subtract
  Eigen::MatrixXd J_rel_body = J_body - X_rel_body.matrix() * J_rel;

  torqueTarget_ = J_rel_body.transpose()
                      * (stiffness_.asDiagonal() * posError_.vector() + damping_.asDiagonal() * velError_.vector())
                  + torqueFeedforward_;

  std::vector<std::vector<double>> torque_vector = robots_.robot(rIndex_).mbc().jointTorque;

  Eigen::VectorXd torque_target_full = Eigen::VectorXd::Zero(robots_.robot(rIndex_).mb().nrDof());
  torque_target_full.tail(nbActuatedJoints) = torqueTarget_;

  torque_vector = rbd::sVectorToDof(robots_.robot(rIndex_).mb(), torque_target_full);

  TorqueTask::torque(torque_vector);
  TorqueTask::update(solver);
}

void TorquePDRelativeCartesianTask::setStiffness(double stiffness)
{
  stiffness_ = Eigen::Vector6d::Constant(stiffness);
}

void TorquePDRelativeCartesianTask::setDamping(double damping)
{
  damping_ = Eigen::Vector6d::Constant(damping);
}

void TorquePDRelativeCartesianTask::setStiffness(const Eigen::Vector6d & stiffness)
{
  stiffness_ = stiffness;
}

void TorquePDRelativeCartesianTask::setDamping(const Eigen::Vector6d & damping)
{
  damping_ = damping;
}

void TorquePDRelativeCartesianTask::setPosTarget(const sva::PTransformd & xd)
{
  posTarget_ = xd;
}

void TorquePDRelativeCartesianTask::setVelTarget(const sva::MotionVecd & xd_dot)
{
  velTarget_ = xd_dot;
}

void TorquePDRelativeCartesianTask::setTorqueFeedforward(const Eigen::VectorXd & tau_ff)
{
  if(tau_ff.size() != nbActuatedJoints)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[TorquePDRelativeCartesianTask] Torque feedforward vector size should be {}, got {}", nbActuatedJoints,
        tau_ff.size());
  }
  torqueFeedforward_ = tau_ff;
}

const Eigen::Vector6d & TorquePDRelativeCartesianTask::stiffness() const
{
  return stiffness_;
}

const Eigen::Vector6d & TorquePDRelativeCartesianTask::damping() const
{
  return damping_;
}

const sva::PTransformd & TorquePDRelativeCartesianTask::posTarget() const
{
  return posTarget_;
}

const sva::MotionVecd & TorquePDRelativeCartesianTask::velTarget() const
{
  return velTarget_;
}

const Eigen::VectorXd & TorquePDRelativeCartesianTask::torqueFeedforward() const
{
  return torqueFeedforward_;
}

void TorquePDRelativeCartesianTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
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

  gui.addElement({"Tasks", name_, "Position Target"},
                 mc_rtc::gui::Transform(
                     "pos_target", [this]() { return this->posTarget() * relative_->position(); },
                     [this](const sva::PTransformd & X_0_target)
                     { this->setPosTarget(X_0_target * relative_->position().inv()); }),
                 mc_rtc::gui::Transform("pos", [this]() { return this->frame()->position(); }));

  gui.addElement({"Tasks", name_, "Velocity Target"}, mc_rtc::gui::ArrayInput("Velocity Target", velTarget_));

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

    auto updateTorqueFeedforward = [this](int i, double v)
    {
      this->torqueFeedforward_[i] = v;
      setTorqueFeedforward(torqueFeedforward_);
    };

    gui.addElement({"Tasks", name_, "Torque Feedforward"},
                   mc_rtc::gui::NumberSlider(
                       j.name(), [this, i]() { return this->torqueFeedforward_[i]; },
                       [i, updateTorqueFeedforward](double v) { updateTorqueFeedforward(i, v); },
                       -robot.tl()[jIndex][0], robot.tu()[jIndex][0]));
    i++;
  }
  TorqueTask::addToGUI(gui);
}

void TorquePDRelativeCartesianTask::addToLogger(mc_rtc::Logger & logger)
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
