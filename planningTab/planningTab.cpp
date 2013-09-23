/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * 
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "planningTab.h"

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <iostream>

#include <dart/collision/CollisionDetector.h>
#include <dart/dynamics/Skeleton.h>
#include <dart/constraint/ConstraintDynamics.h>
#include <dart/dynamics/BoxShape.h>
#include <dart/dynamics/GenCoord.h>
#include <dart/dynamics/Joint.h>
#include <dart/planning/PathPlanner.h>
#include <dart/planning/PathShortener.h>
#include <dart/planning/PathFollowingTrajectory.h>
#include "Controller.h"

using namespace std;

// Define IDs for buttons
enum DynamicSimulationTabEvents {
  id_button_SetStart = 8345,
  id_button_SetGoal,
  id_button_SetPredefStart,
  id_button_SetPredefGoal,
  id_button_RelocateObjects,
  id_button_ShowStart,
  id_button_ShowGoal,
  id_button_Plan
};

// Handler for events
BEGIN_EVENT_TABLE(planningTab, wxPanel)
EVT_COMMAND (id_button_SetStart, wxEVT_COMMAND_BUTTON_CLICKED, planningTab::onButtonSetStart)
EVT_COMMAND (id_button_SetGoal, wxEVT_COMMAND_BUTTON_CLICKED, planningTab::onButtonSetGoal)
EVT_COMMAND (id_button_SetPredefStart, wxEVT_COMMAND_BUTTON_CLICKED, planningTab::onButtonSetPredefStart)
EVT_COMMAND (id_button_SetPredefGoal, wxEVT_COMMAND_BUTTON_CLICKED, planningTab::onButtonSetPredefGoal)
EVT_COMMAND (id_button_RelocateObjects, wxEVT_COMMAND_BUTTON_CLICKED, planningTab::onButtonRelocateObjects)
EVT_COMMAND (id_button_ShowStart, wxEVT_COMMAND_BUTTON_CLICKED, planningTab::onButtonShowStart)
EVT_COMMAND (id_button_ShowGoal, wxEVT_COMMAND_BUTTON_CLICKED, planningTab::onButtonShowGoal)
EVT_COMMAND (id_button_Plan, wxEVT_COMMAND_BUTTON_CLICKED, planningTab::onButtonPlan)
END_EVENT_TABLE()

IMPLEMENT_DYNAMIC_CLASS(planningTab, GRIPTab)

planningTab::planningTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, long style) :
  GRIPTab(parent, id, pos, size, style)
{
  // Create user interface
  wxSizer* sizerFull= new wxBoxSizer(wxHORIZONTAL);
  
  wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("Setup planning problem"));
  wxStaticBox* ss2Box = new wxStaticBox(this, -1, wxT("Check"));
  wxStaticBox* ss3Box = new wxStaticBox(this, -1, wxT("Execute"));
  
  wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);
  wxStaticBoxSizer* ss2BoxS = new wxStaticBoxSizer(ss2Box, wxVERTICAL);
  wxStaticBoxSizer* ss3BoxS = new wxStaticBoxSizer(ss3Box, wxVERTICAL);

  ss1BoxS->Add(new wxButton(this, id_button_SetStart, wxT("Set Start Conf")), 0, wxALL, 1); 
  ss1BoxS->Add(new wxButton(this, id_button_SetGoal, wxT("Set Goal Conf")), 0, wxALL, 1); 
  ss1BoxS->Add(new wxButton(this, id_button_SetPredefStart, wxT("Set Predef Start")), 0, wxALL, 1); 
  ss1BoxS->Add(new wxButton(this, id_button_SetPredefGoal, wxT("Set Predef Goal")), 0, wxALL, 1); 
  ss1BoxS->Add(new wxButton(this, id_button_RelocateObjects, wxT("Relocate objects")), 0, wxALL, 1);
  ss2BoxS->Add(new wxButton(this, id_button_ShowStart, wxT("Show Start")), 0, wxALL, 1); 
  ss2BoxS->Add(new wxButton(this, id_button_ShowGoal, wxT("Show Goal")), 0, wxALL, 1); 
  ss3BoxS->Add(new wxButton(this, id_button_Plan, wxT("Do Planning")), 0, wxALL, 1); 

  sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 6);
  sizerFull->Add(ss2BoxS, 1, wxEXPAND | wxALL, 6);
  sizerFull->Add(ss3BoxS, 1, wxEXPAND | wxALL, 6);

  SetSizer(sizerFull);

  // Set predefined start and goal configuration
  mPredefStartConf.resize(6);
  mPredefGoalConf.resize(6);
  mPredefStartConf << -0.858702, -0.674395, 0.0, -0.337896, 0.0, 0.0;
  mPredefGoalConf << -0.69115, 0.121475, 0.284977, -1.02486, 0.0, 0.0;
  mStartConf = mPredefStartConf;
  mGoalConf = mPredefGoalConf;
}


/// Gets triggered after a world is loaded
void planningTab::GRIPEventSceneLoaded() {
  mRobot = mWorld->getSkeleton("GolemHubo");

  // Set initial configuration for the legs
  int legDofsArray[] = {19, 20, 23, 24, 27, 28};
  vector<int> legDofs(legDofsArray, legDofsArray + 6);
  Eigen::VectorXd legValues(6);
  legValues << -10.0, -10.0, 20.0, 20.0, -10.0, -10.0;
  legValues *= M_PI / 180.0;
  mRobot->setConfig(legDofs, legValues);

  // Define right arm nodes
  const string armNodes[] = {"Body_RSP", "Body_RSR", "Body_RSY", "Body_REP", "Body_RWY", "Body_RWP"}; 
  mArmDofs.resize(6);
  for(int i = 0; i < mArmDofs.size(); i++) {
    mArmDofs[i] = mRobot->getBodyNode(armNodes[i])->getParentJoint()->getGenCoord(0)->getSkeletonIndex();
  }
}


/// Before each simulation step we set the torques the controller applies to the joints
void planningTab::GRIPEventSimulationBeforeTimestep() {
  Eigen::VectorXd torques = mController->getTorques(mRobot->get_q(), mRobot->get_dq(), mWorld->getTime());
  mRobot->setInternalForces(torques);
}


/// Set start configuration to the configuration the arm is currently in
void planningTab::onButtonSetStart(wxCommandEvent & _evt) {
  if(!mWorld || mWorld->getNumSkeletons() < 1) {
    cout << "No world loaded or world does not contain a robot." << endl;
    return;
  }

  mStartConf = mRobot->getConfig(mArmDofs);
  cout << "Start Configuration: " << mStartConf.transpose() << endl;
}


/// Set goal configuration to the configuration the arm is currently in
void planningTab::onButtonSetGoal(wxCommandEvent & _evt) {
  if(!mWorld || mWorld->getNumSkeletons() < 1) {
    cout << "No world loaded or world does not contain a robot." << endl;
    return;
  }

  mGoalConf = mRobot->getConfig(mArmDofs);
  cout << "Goal Configuration: " << mGoalConf.transpose() << endl;
}


/// Reset start configuration to the predefined one
void planningTab::onButtonSetPredefStart(wxCommandEvent & _evt) {
  mStartConf = mPredefStartConf;
}


/// Reset goal configuration to the predefined one
void planningTab::onButtonSetPredefGoal(wxCommandEvent & _evt) {
  mGoalConf = mPredefGoalConf;
}


/// Move objects to obstruct the direct path between the predefined start and goal configurations
void planningTab::onButtonRelocateObjects(wxCommandEvent & _evt) {

  dart::dynamics::Skeleton* orangeCube = mWorld->getSkeleton("orangeCube");
  dart::dynamics::Skeleton* yellowCube = mWorld->getSkeleton("yellowCube");
  
  if(!orangeCube || !yellowCube) {
    cout << "Did not find orange or yellow object. Exiting and no moving anything" << endl;
    return;
  }
  
  Eigen::Matrix<double, 6, 1> pose; 
  pose << 0.0, 0.0, 0.0, 0.30, -0.30, 0.83;
  orangeCube->setConfig(pose);
  pose << 0.0, 0.0, 0.0, 0.30, -0.30, 0.935;
  yellowCube->setConfig(pose);

  viewer->DrawGLScene();
}


/// Show the currently set start configuration
void planningTab::onButtonShowStart(wxCommandEvent & _evt) {
  cout << "Showing start conf for right arm: " << mStartConf.transpose() << endl;
  mRobot->setConfig(mArmDofs, mStartConf);
  viewer->DrawGLScene();
}


/// Show the currently set goal configuration
void planningTab::onButtonShowGoal(wxCommandEvent & _evt) {
  cout << "Showing goal conf for right arm: " << mGoalConf.transpose() << endl;
  mRobot->setConfig(mArmDofs, mGoalConf);
  viewer->DrawGLScene();
}


/// Set initial dynamic parameters and call planner and controller
void planningTab::onButtonPlan(wxCommandEvent & _evt) {

  // Store the actuated joints (all except the first 6 which are only a convenience to locate the robot in the world)
  std::vector<int> actuatedDofs(mRobot->getNumGenCoords() - 6);
  for(unsigned int i = 0; i < actuatedDofs.size(); i++) {
    actuatedDofs[i] = i + 6;
  }
  
  // Deactivate collision checking between the feet and the ground during planning
  dart::dynamics::Skeleton* ground = mWorld->getSkeleton("ground");
  mWorld->getCollisionHandle()->getCollisionChecker()->disablePair(mRobot->getBodyNode("Body_LAR"), ground->getRootBodyNode());
  mWorld->getCollisionHandle()->getCollisionChecker()->disablePair(mRobot->getBodyNode("Body_RAR"), ground->getRootBodyNode());
  
  // Define PD controller gains
  Eigen::VectorXd kI = 100.0 * Eigen::VectorXd::Ones(mRobot->getNumGenCoords());
  Eigen::VectorXd kP = 500.0 * Eigen::VectorXd::Ones(mRobot->getNumGenCoords());
  Eigen::VectorXd kD = 100.0 * Eigen::VectorXd::Ones(mRobot->getNumGenCoords());

  // Define gains for the ankle PD
  std::vector<int> ankleDofs(2);
  ankleDofs[0] = 27;
  ankleDofs[1] = 28;
  const Eigen::VectorXd anklePGains = -1000.0 * Eigen::VectorXd::Ones(2);
  const Eigen::VectorXd ankleDGains = -200.0 * Eigen::VectorXd::Ones(2);

  // Set robot to start configuration
  mRobot->setConfig(mArmDofs, mStartConf);

  // Create controller
  mController = new Controller(mRobot, actuatedDofs, kP, kD, ankleDofs, anklePGains, ankleDGains);

  // Call path planner
  dart::planning::PathPlanner<> pathPlanner(*mWorld);
  std::list<Eigen::VectorXd> path;
  if(!pathPlanner.planPath(mRobot, mArmDofs, mStartConf, mGoalConf, path)) {
    std::cout << "Path planner could not find a path." << std::endl;
  }
  else {
    // Call path shortener
    dart::planning::PathShortener pathShortener(mWorld, mRobot, mArmDofs);
    pathShortener.shortenPath(path);

    // Convert path into time-parameterized trajectory satisfying acceleration and velocity constraints
    const Eigen::VectorXd maxVelocity = 0.6 * Eigen::VectorXd::Ones(mArmDofs.size());
    const Eigen::VectorXd maxAcceleration = 0.6 * Eigen::VectorXd::Ones(mArmDofs.size());
    dart::planning::Trajectory* trajectory = new dart::planning::PathFollowingTrajectory(path, maxVelocity, maxAcceleration);
    std::cout << "-- Trajectory duration: " << trajectory->getDuration() << endl;
    mController->setTrajectory(trajectory, 0.0, mArmDofs);
  }
  
  // Reactivate collision of feet with floor
  mWorld->getCollisionHandle()->getCollisionChecker()->enablePair(mRobot->getBodyNode("Body_LAR"), ground->getRootBodyNode());
  mWorld->getCollisionHandle()->getCollisionChecker()->enablePair(mRobot->getBodyNode("Body_RAR"), ground->getRootBodyNode());
}

// Local Variables:
// c-basic-offset: 2
// End:
