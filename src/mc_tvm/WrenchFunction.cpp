/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/DynamicFunction.h>
#include <mc_tvm/WrenchFunction.h>

#include <mc_tvm/Robot.h>
#include <mc_tvm/RobotFrame.h>

#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/RobotFrame.h>

#include <Eigen/Cholesky>

namespace mc_tvm
{

WrenchFunction::WrenchFunction(const mc_rbdyn::RobotFrame & frame)
: tvm::function::abstract::LinearFunction(6), frame_(frame), tvm_frame_(frame.tvm_frame()), robot_(frame.robot()),
  tvm_robot_(robot_.tvmRobot()), frameJac_(tvm_frame_.rbdJacobian()), shortJacMat_(6, frameJac_.dof()),
  jacMat_(6, robot_.mb().nrDof()), dynamicJacTransposeMat_(robot_.mb().nrDof(), 6)
{
  reset();
  registerUpdates(Update::B, &WrenchFunction::updateb);
  registerUpdates(Update::Jacobian, &WrenchFunction::updateJacobian);
  addOutputDependency<WrenchFunction>(Output::B, Update::B);
  addOutputDependency<WrenchFunction>(Output::Jacobian, Update::Jacobian);

  auto & robot = frame_->robot();
  auto & tvm_robot = robot.tvmRobot();

  addInputDependency<WrenchFunction>(Update::Jacobian, tvm_robot, Robot::Output::H);
  addInternalDependency<WrenchFunction>(Update::Jacobian, Update::B);
  // addInputDependency<WrenchFunction>(Update::B, tvm_frame_, mc_tvm::RobotFrame::Output::Jacobian);
  addInputDependency<WrenchFunction>(Update::Jacobian, tvm_frame_, mc_tvm::RobotFrame::Output::Jacobian);
  addInputDependency<WrenchFunction>(Update::B, tvm_robot, Robot::Output::C);
  addInputDependency<WrenchFunction>(Update::B, tvm_robot, Robot::Output::tau);
  addVariable(tvm::dot(tvm_robot.q(), 2), true);
  addVariable(tvm_robot.tau(), true);
  jacobian_[tvm_robot.tau().get()] = Eigen::MatrixXd::Identity(robot.mb().nrDof(), robot.mb().nrDof());
  jacobian_[tvm_robot.tau().get()].properties(tvm::internal::MatrixProperties::IDENTITY);
  velocity_.setZero();
}

void WrenchFunction::reset()
{
  wrench_ = currentWrench();
}

sva::ForceVecd WrenchFunction::currentWrench()
{
  computeDynamicJacobian();
  Eigen::VectorXd wrenchVec_ = dynamicJacTransposeMat_ * tvm_robot_.tau()->value();
  return sva::ForceVecd(wrenchVec_.head<3>(), wrenchVec_.tail<3>());
}

void WrenchFunction::computeDynamicJacobian()
{
  shortJacMat_ = frameJac_.jacobian(robot_.mb(), robot_.mbc(), tvm_frame_.position());
  frameJac_.fullJacobian(robot_.mb(), shortJacMat_, jacMat_);

  const Eigen::MatrixXd & H = tvm_robot_.H();

  // 1. Factorize H (SPD)
  Eigen::LDLT<Eigen::MatrixXd> H_ldlt(H);

  // 2. Compute M^{-1} J^T
  Eigen::MatrixXd MinvJt = H_ldlt.solve(jacMat_.transpose());

  // 3. Compute Lambda = (J M^{-1} J^T)^{-1} using LDLT
  Eigen::MatrixXd JMJM = jacMat_ * MinvJt;
  Eigen::MatrixXd lambda = JMJM.ldlt().solve(Eigen::MatrixXd::Identity(JMJM.rows(), JMJM.cols()));

  // 4. Compute dynamically consistent J^#
  dynamicJacTransposeMat_ = MinvJt * lambda;
}

void WrenchFunction::updateJacobian()
{
  computeDynamicJacobian();
  splitJacobian(dynamicJacTransposeMat_.transpose(), tvm_robot_.tau());
}

void WrenchFunction::updateb() // Ax + b = 0
{
  b_ = -wrench_.vector();
}

} // namespace mc_tvm
