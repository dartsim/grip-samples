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
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <iostream>
using namespace std;

#include <Tabs/AllTabs.h>
#include <GRIPApp.h>


// **********************
// Dynamics Stuff
#include <collision/CollisionShapes.h>
#include <collision/CollisionSkeleton.h>
#include <dynamics/SkeletonDynamics.h>
#include <dynamics/ContactDynamics.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/ShapeCube.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
#include <robotics/Object.h>
#include <robotics/Robot.h>

// Planning and controller
#include <planning/PathPlanner.h>
#include <planning/Trajectory.h>
#include "Controller.h"
// **********************


/** Events */
enum DynamicSimulationTabEvents {
  id_button_AddFloor = 8345,
  id_button_StartSim,
  id_button_InitSettings,
  id_button_SetTimeline
};

/** Handler for events **/
BEGIN_EVENT_TABLE(planningTab, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, planningTab::OnButton)
EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, planningTab::OnSlider)
END_EVENT_TABLE()


// Class constructor for the tab: Each tab will be a subclass of GRIPTab
IMPLEMENT_DYNAMIC_CLASS(planningTab, GRIPTab)

// Define right arm nodes
string const planningTab::mRA_Nodes[mRA_NumNodes] = {"Body_RSP", "Body_RSR", "Body_RSY", "Body_REP", "Body_RWY", "rightUJoint", "rightPalmDummy"}; 


/**
 * @function planningTab
 * @brief Constructor
 */
planningTab::planningTab(wxWindow *parent, const wxWindowID id,
			 const wxPoint& pos, const wxSize& size, long style) :
  GRIPTab(parent, id, pos, size, style) {
  sizerFull = new wxBoxSizer(wxHORIZONTAL);
  
  // Create Static boxes (outline of your Tab)
  wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("Simulation"));
  
  // Create sizers for these static boxes
  wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);
  
  // Buttons
  ss1BoxS->Add(new wxButton(this, id_button_AddFloor, wxT("Add floor")), 0, wxALL, 1); 
  ss1BoxS->Add(new wxButton(this, id_button_InitSettings, wxT("Init Settings and Planning")), 0, wxALL, 1); 
  ss1BoxS->Add(new wxButton(this, id_button_SetTimeline, wxT("Set Timeline")), 0, wxALL, 1); 
  
  
  // Add the boxes to their respective sizers
  sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 6);
  
  SetSizer(sizerFull);
  
  // Initialization
  mCurrentFrame = 0;

}


/**
 * @function OnButton
 * @brief Handles button events
 */
void planningTab::OnButton(wxCommandEvent & _evt) {
  
  int slnum = _evt.GetId();
  
  switch( slnum ) {
    
    // Add Floor for Hubo to fall on :)
  case id_button_AddFloor: {
    addFloor();
  }
    break;
    
  
    // Start Dynamic Simulation
  case id_button_InitSettings: {
    initSettings();    
  }
    break;
    
    // Set Timeline
  case id_button_SetTimeline: {
    setTimeline();
  }    
    break;
    
  default: {
    printf("Default button \n");
  }
  }
}

/**
 * @function addFloor
 * @brief Add a floor *immobile* object
 */
void planningTab::addFloor() {

  robotics::Object* ground = new robotics::Object();
  ground->setName("ground");
  ground->addDefaultRootNode();
  dynamics::BodyNodeDynamics* node = new dynamics::BodyNodeDynamics();
  node->setShape( new kinematics::ShapeCube( Eigen::Vector3d( 10.0, 10.0, 0.0001), 1.0));

  kinematics::Joint* joint = new kinematics::Joint( ground->getRoot(), node );
  ground->addNode( node );
  ground->initSkel();
  ground->update();
  ground->setImmobileState( true );
  mWorld->addObject( ground );
  mWorld->rebuildCollision();

  treeView->CreateFromWorld();

  printf("-- Added floor \n" );    
}

/**
 * @function initSettings
 */
void planningTab::initSettings() {

  // Get the ground object index
  int groundIndex = -1;
  for( int i = 0; i < mWorld->getNumObjects(); ++i ) {
    if( mWorld->getObject(i)->getName() == "ground" ) {
      groundIndex = i; break;
    }
  }
  if( groundIndex == -1 ) { printf("I did not find the floor. EXIT! \n"); return; }
  else { printf("-- Ground is object %d \n", groundIndex);}

  // Get the robot (in this case there is only one)
  int robotIndex = 0;

  // Get the DOF indices for the Right Arm
  std::vector<int> trajectoryDofs( mRA_NumNodes );
  for(int i = 0; i < mRA_NumNodes; i++) {
    trajectoryDofs[i] = mWorld->getRobot(robotIndex)->getNode(mRA_Nodes[i].c_str())->getDof(0)->getSkelIndex();
  }

  // Store the actuated DOF (do not consider the first 6 since they are not actually actuated)
  std::vector<int> actuatedDofs(mWorld->getRobot(robotIndex)->getNumDofs() - 6);
  for(unsigned int i = 0; i < actuatedDofs.size(); i++) {
    actuatedDofs[i] = i + 6;
  }
  
  // Set an initial configuration for the robot
  mWorld->getRobot(robotIndex)->getDof(19)->setValue(-10.0 * M_PI/180.0);
  mWorld->getRobot(robotIndex)->getDof(20)->setValue(-10.0 * M_PI/180.0);
  mWorld->getRobot(robotIndex)->getDof(23)->setValue(20.0 * M_PI/180.0);
  mWorld->getRobot(robotIndex)->getDof(24)->setValue(20.0 * M_PI/180.0);
  mWorld->getRobot(robotIndex)->getDof(27)->setValue(-10.0 * M_PI/180.0);
  mWorld->getRobot(robotIndex)->getDof(28)->setValue(-10.0 * M_PI/180.0);
  // Update the Viewer
  mWorld->getRobot(robotIndex)->update();
  viewer->DrawGLScene();
  
  // Deactivate collision checking between the feet and the ground during planning
  mWorld->mCollisionHandle->getCollisionChecker()->deactivatePair(mWorld->getRobot(robotIndex)->getNode("leftFoot"), mWorld->getObject(groundIndex)->getNode(1));
  mWorld->mCollisionHandle->getCollisionChecker()->deactivatePair(mWorld->getRobot(robotIndex)->getNode("rightFoot"), mWorld->getObject(groundIndex)->getNode(1));
  
  // Set the gains for the PD controller (Body)
  Eigen::VectorXd kI = 100.0 * Eigen::VectorXd::Ones(mWorld->getRobot(robotIndex)->getNumDofs());
  Eigen::VectorXd kP = 500.0 * Eigen::VectorXd::Ones(mWorld->getRobot(robotIndex)->getNumDofs());
  Eigen::VectorXd kD = 100.0 * Eigen::VectorXd::Ones(mWorld->getRobot(robotIndex)->getNumDofs());

  // Set the gains for the ankle PD controller
  std::vector<int> ankleDofs(2);
  ankleDofs[0] = 27;
  ankleDofs[1] = 28;
  const Eigen::VectorXd anklePGains = -1000.0 * Eigen::VectorXd::Ones(2);
  const Eigen::VectorXd ankleDGains = -2000.0 * Eigen::VectorXd::Ones(2);

  // Set the controller
  mController = new planning::Controller(mWorld->getRobot(robotIndex),
                                         actuatedDofs,
                                         kP,
                                         kD, 
					 ankleDofs,
                                         anklePGains,
                                         ankleDGains,
                                         mWorld->mTimeStep);

  // Set the planner
  planning::PathPlanner<> pathPlanner(*mWorld);

  // Set goal right arm configuration
  Eigen::VectorXd goalConf(mRA_NumNodes);
  goalConf << 0.0, -M_PI / 2.0, 0.0, -M_PI / 2.0, 0.0, 0.0, 0.0;
  Eigen::VectorXd startConf(mRA_NumNodes);
  startConf << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // Find the path
  std::list<Eigen::VectorXd> path;
  if(!pathPlanner.planPath(robotIndex, trajectoryDofs, startConf, goalConf, path)) {
    std::cout << "<!> Path planner could not find a path" << std::endl;
  }
  // Create the trajectory
  else {
    // Set maximum velocities and acceleration for the right arm joints
    const Eigen::VectorXd maxVelocity = 0.3 * Eigen::VectorXd::Ones(mRA_NumNodes);
    const Eigen::VectorXd maxAcceleration = 0.3 * Eigen::VectorXd::Ones(mRA_NumNodes);
    planning::Trajectory* trajectory = new planning::Trajectory(path, maxVelocity, maxAcceleration);
    std::cout << "-- Trajectory duration: " << trajectory->getDuration() << std::endl;
    mController->setTrajectory(trajectory, 0.1, trajectoryDofs);
  }
  
  // Re-activate collision for foot and ground
  mWorld->mCollisionHandle->getCollisionChecker()->activatePair(mWorld->getRobot(robotIndex)->getNode("leftFoot"), mWorld->getObject(groundIndex)->getNode(1));
  mWorld->mCollisionHandle->getCollisionChecker()->activatePair(mWorld->getRobot(robotIndex)->getNode("rightFoot"), mWorld->getObject(groundIndex)->getNode(1));
  
}

/**
 * @function GRIPEventSimulationBeforeTimeStep
 * @brief Before each sim step we must set the internal forces 
 */
void planningTab::GRIPEventSimulationBeforeTimestep() {

  int robotIndex = 0;
  mWorld->getRobot(robotIndex)->setInternalForces(mController->getTorques(mWorld->getRobot(robotIndex)->getPose(), mWorld->getRobot(robotIndex)->getQDotVector(), mWorld->mTime));
 
}

/**
 * @function GRIPEventSimulationAfterTimeStep
 * @brief After 30 sim steps we save frames for future playback
 */
void planningTab::GRIPEventSimulationAfterTimestep() {
    std::cout << "  Time: " << mWorld->mTime << std::endl << std::flush;
  mCurrentFrame++;
  if( mCurrentFrame % 30 == 0 ) {
    bake();
  }
}


/**
 * @function setTimeline
 */
void planningTab::setTimeline() {

  int numsteps = mBakedStates.size();
  
  double increment = mWorld->mTimeStep;
  double totalTime = mWorld->mTime;
  
  cout << "-->(+) Updating Timeline - Increment: " << increment << " Total T: " << totalTime << " Steps: " << numsteps << endl;
  
  frame->InitTimer( string("Planner"),increment );
  
  // Set the Time slider with the saved simulated frames
  for( int i = 0; i < numsteps; ++i ) {
    retrieveBakedState( i );
    for (int j = 0; j < mWorld->getNumRobots(); j++) {
      mWorld->getRobot(j)->update();
    }
    for (int j = 0; j < mWorld->getNumObjects(); j++) {
      mWorld->getObject(j)->update();
    }
    frame->AddWorld( mWorld );
    
  }
  printf("-- Finished setting timeline \n");

} 


/**
 * @function bake
 * @brief
 */
void planningTab::bake() {

    VectorXd state(mWorld->mIndices.back());
    for(int i = 0; i < mWorld->getNumSkeletons(); i++) {
        state.segment(mWorld->mIndices[i], mWorld->mDofs[i].size()) = mWorld->mDofs[i];
    }
    mBakedStates.push_back(state);
}

/**
 * @function retrieveBakedState
 * @brief Set the world to the saved state at frame _frame
 */
void planningTab::retrieveBakedState( int _frame ) {

  for (int i = 0; i < mWorld->getNumSkeletons(); i++) {
    int start = mWorld->mIndices[i];
    int size = mWorld->mDofs[i].size();
    mWorld->getSkeleton(i)->setPose(mBakedStates[_frame].segment(start, size), false, false);
  }

}

/**
 * @function OnSlider
 * @brief Handles slider changes
 */
void planningTab::OnSlider(wxCommandEvent &evt) {

}


// This function is called when an object is selected in the Tree View or other
// global changes to the GRIP world. Use this to capture events from outside the tab.
void planningTab::GRIPStateChange() {
  if(selectedTreeNode==NULL){
    return;
  }
  
  string statusBuf;
  string buf, buf2;
  switch (selectedTreeNode->dType) {
  case Return_Type_Object:
    statusBuf = " Selected Object: ";
    buf = "You clicked on object: ";
    
    break;
  case Return_Type_Robot:
    statusBuf = " Selected Robot: ";
    buf = "You clicked on robot: ";
    
    break;
  case Return_Type_Node:
    statusBuf = " Selected Link:  of Robot: ";
    buf = " Link:  of Robot: ";
    // Do something here if you want to.  you get the idea...
    
    break;
  default:
    fprintf(stderr, "someone else's problem.");
    assert(0);
    exit(1);
  }
  //frame->SetStatusText(wxString(statusBuf.c_str(), wxConvUTF8));
  //sizerFull->Layout();
}


