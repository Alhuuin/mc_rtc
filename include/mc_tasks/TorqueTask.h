#pragma once

#include <mc_tasks/MetaTask.h>

#include <mc_rtc/logging.h>
#include <mc_rtc/void_ptr.h>

#include <Tasks/QPTasks.h>

namespace mc_tasks
{

struct MC_TASKS_DLLAPI TorqueTask : public MetaTask
{
public:
  /*! \brief Constructor
   *
   * \param robot Robot controlled by this task
   *
   * \param robotIndex Index of the robot controlled by this task
   *
   * \param weight Task weight
   */
  TorqueTask(const mc_rbdyn::Robots & robots,
             unsigned int robotIndex,
             double weight = 1000.0);

  /*! \brief Reset the task
   *
   * Set the task objective to the current joint torques
   */
  void reset() override;

  /*! \brief Get the torque target */
  const Eigen::VectorXd & torques() const;

  /*! \brief Set the torque target
   *
   * \param t Joint torques
   */
  void torques(const Eigen::VectorXd & t);

  /*! \brief Set the torque target for a specific joint
   *
   * \param jointName Name of the joint
   *
   * \param t Joint torque (will be applied to all DOFs of the joint)
   */
  void torque(const std::string & jointName, double t);

  /*! \brief Set the torque target for a specific joint with multiple DOFs
   *
   * \param jointName Name of the joint
   *
   * \param t Vector of joint torques, one for each DOF
   */
  void torque(const std::string & jointName, const Eigen::VectorXd & t);

  /*! \brief Get the current joint torques */
  Eigen::VectorXd currentTorques() const;

  /*! \brief Compute the task error */
  Eigen::VectorXd eval() const override;

  /*! \brief Compute the task speed */
  Eigen::VectorXd speed() const override;

  // Définir le masque d'articulations
  void selectActiveJoints(const std::vector<std::string> & activeJointsName);

  const mc_rbdyn::Robot & robot() const { return robots_.robot(rIndex_); }

  /*! \brief Load parameters from a Configuration object */
  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;
  
  /*! \brief Set the task dimensional weight
   *
   * For simple cases (using 0/1 as weights) prefer \ref selectActiveJoints or \ref selectUnactiveJoints which are
   * simpler to use
   */
  void dimWeight(const Eigen::VectorXd & dimW) override;
  
  Eigen::VectorXd dimWeight() const override;
  
  /*! \brief Select active joints for this task
   *
   * Manipulate \ref dimWeight() to achieve its effect
   */
  void selectActiveJoints(mc_solver::QPSolver & solver,
                          const std::vector<std::string> & activeJointsName,
                          const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs = {}) override;
  
  /*! \brief Same as selectInactiveJoints but with a spelling mistake
   *
   * Concerved because it was the default spelling in other tasks. Will call selectInactiveJoints and display a warning
   */
  void selectUnactiveJoints(mc_solver::QPSolver & solver,
                            const std::vector<std::string> & unactiveJointsName,
                            const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs = {}) override;
  
  
  /*! \brief Select inactive joints for this task
   *
   * Manipulate \ref dimWeight() to achieve its effect
   */
  void selectInactiveJoints(mc_solver::QPSolver & solver,
                            const std::vector<std::string> & unactiveJointsName,
                            const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs = {});
  /*! \brief Reset the joint selector effect
   *
   * Reset dimWeight to ones
   */
  void resetJointsSelector(mc_solver::QPSolver & solver) override;
  
  /** Get the task weight */
  double weight() const;

  /** Set the task weight */
  void weight(double w);

protected:
  /*! \brief Add the task to the logger */
  void addToLogger(mc_rtc::Logger & logger) override;

  /*! \brief Add the task to the GUI */
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;
  
  /*! \brief Add the task to the solver */
  void addToSolver(mc_solver::QPSolver & solver) override;

  /*! \brief Remove the task from the solver */
  void removeFromSolver(mc_solver::QPSolver & solver) override;

  /*! \brief Update the task */
  void update(mc_solver::QPSolver & solver) override;

  const mc_rbdyn::Robots & robots_;
  unsigned int rIndex_;
  mc_rtc::void_ptr pt_;  // Tâche de bas niveau
  Eigen::VectorXd targetTorques_;
  bool inSolver_ = false;
};

} //namespace mc_tasks