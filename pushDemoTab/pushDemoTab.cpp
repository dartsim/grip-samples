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

#include "pushDemoTab.h"

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
  id_button_SetStart,
  id_button_SetGoal,
  id_button_StartSim, // just to be safe
  id_button_Planning,
  id_button_Play,
  id_button_FrameForward,
  id_button_FrameBackward,
  id_button_ApplyForce1
};

/** Handler for events **/
BEGIN_EVENT_TABLE(pushDemoTab, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, pushDemoTab::OnButton)
EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, pushDemoTab::OnSlider)
END_EVENT_TABLE()


// Class constructor for the tab: Each tab will be a subclass of GRIPTab
IMPLEMENT_DYNAMIC_CLASS(pushDemoTab, GRIPTab)

// Define right arm nodes
string const pushDemoTab::mRA_TrajNodes[mRA_NumNodes] = {"Body_RSP", "Body_RSR", "Body_RSY", "Body_REP", "Body_RWY", "rightUJoint", "rightPalmDummy"}; 

/**
 * @function pushDemoTab
 * @brief Constructor
 */
pushDemoTab::pushDemoTab(wxWindow *parent, const wxWindowID id,
		const wxPoint& pos, const wxSize& size, long style) :
  GRIPTab(parent, id, pos, size, style) {
  sizerFull = new wxBoxSizer(wxHORIZONTAL);
  
  // Create Static boxes (outline of your Tab)
  wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("Simulation"));
  
  // Create sizers for these static boxes
  wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);

  // Start - Stop buttons
  ss1BoxS->Add(new wxButton(this, id_button_AddFloor, wxT("Add floor")), 0, wxALL, 1); 
  ss1BoxS->Add(new wxButton(this, id_button_SetStart, wxT("Set Start")), 0, wxALL, 1); 
  ss1BoxS->Add(new wxButton(this, id_button_SetGoal, wxT("Set Goal")), 0, wxALL, 1); 
  ss1BoxS->Add(new wxButton(this, id_button_Planning, wxT("Do Planning")), 0, wxALL, 1); 
  ss1BoxS->Add(new wxButton(this, id_button_StartSim, wxT("Simulate")), 0, wxALL, 1); 
  
  // Add the boxes to their respective sizers
  sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 6);

  SetSizer(sizerFull);
  
  // Initialize Dynamic Variables

  mSimFrame = 0;
  mPlayFrame = 0;
  mMovieFrame = 0;
  mDisplayTimeout = 1000.0 / 30.0;
  
}


/**
 * @function OnButton
 * @brief Handles button events
 */
void pushDemoTab::OnButton(wxCommandEvent & _evt) {
  
  int slnum = _evt.GetId();
  
  switch( slnum ) {
  
    // Add Floor for Hubo to fall on :)
  case id_button_AddFloor: {
    addFloor();
  }
    break;

    // Set Start Arm Configuration
  case id_button_SetStart: {
    
    if( mWorld != NULL ) { 
      if( mWorld->getNumRobots() < 1 ) {  
	printf("No robot in the loaded world, you need one! \n"); break; 
      }
                       
      std::cout<<"* Start Conf: ";
      mStartConf.resize( mRA_NumNodes );
      for(int i = 0; i < mRA_NumNodes; i++) {
	mStartConf[i] = mWorld->getRobot(0)->getNode( mRA_TrajNodes[i].c_str() )->getDof(0)->getValue();
	std::cout<< " "<<mStartConf[i];
      }  
      std::cout<<std::endl;
    }      
    else {  
      std::cout<<"--(!) No world loaded, I cannot set a start position (!)--\n"<<std::endl;
    }
      
  }
    break;


    // Set Goal Arm Configuration
  case id_button_SetGoal: {

    if( mWorld != NULL ) { 
      if( mWorld->getNumRobots() < 1 ) {  
	printf("No robot in the loaded world, you need one! \n"); break; 
      }
                       
      std::cout<<"* Goal Conf: ";
      mGoalConf.resize( mRA_NumNodes );
      for(int i = 0; i < mRA_NumNodes; i++) {
	mGoalConf[i] = mWorld->getRobot(0)->getNode( mRA_TrajNodes[i].c_str() )->getDof(0)->getValue();
	std::cout<< " "<<mGoalConf[i];
      }  
      std::cout<<std::endl;
    }      
    else {  
      std::cout<<"--(!) No world loaded, I cannot set a start position (!)--\n"<<std::endl;
    }

  }
    break;
  
    // Start Dynamic Simulation
  case id_button_Planning: {
    settings();

  }
    break;
    
    // Start Dynamic Simulation
  case id_button_StartSim: {
    simulate();
  }    
    break;

    // Play
  case id_button_Play: {

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
void pushDemoTab::addFloor() {
  printf("Adding floor \n");
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
    
}

/**
 * @function settings
 */
void pushDemoTab::settings() {

  // Get the ground
  int groundIndex = -1;
  for( int i = 0; i < mWorld->getNumObjects(); ++i ) {
    if( mWorld->getObject(i)->getName() == "ground" ) {
      groundIndex = i; break;
    }
  }

  if( groundIndex == -1 ) { printf("I did not find the floor! BAD!! EXITING \n"); return; }
  else { printf("Ground is object %d \n", groundIndex);}

  // Get the robot
  int robotIndex = 0;

  std::vector<int> trajectoryDofs( mRA_NumNodes);
  printf("Trajectory nodes are: ");
  for(int i = 0; i < mRA_NumNodes; i++) {
    trajectoryDofs[i] = mWorld->getRobot(robotIndex)->getNode(mRA_TrajNodes[i].c_str())->getDof(0)->getSkelIndex();
    printf(" %f ", trajectoryDofs[i]);
  }
  printf("\n");
  
  std::vector<int> actuatedDofs(mWorld->getRobot(robotIndex)->getNumDofs() - 6);
  for(unsigned int i = 0; i < actuatedDofs.size(); i++) {
    actuatedDofs[i] = i + 6;
  }
  
  mWorld->getRobot(robotIndex)->getDof(19)->setValue(-10.0 * M_PI/180.0);
  mWorld->getRobot(robotIndex)->getDof(20)->setValue(-10.0 * M_PI/180.0);
  mWorld->getRobot(robotIndex)->getDof(23)->setValue(20.0 * M_PI/180.0);
  mWorld->getRobot(robotIndex)->getDof(24)->setValue(20.0 * M_PI/180.0);
  mWorld->getRobot(robotIndex)->getDof(27)->setValue(-10.0 * M_PI/180.0);
  mWorld->getRobot(robotIndex)->getDof(28)->setValue(-10.0 * M_PI/180.0);
  
  mWorld->getRobot(robotIndex)->update();
  viewer->DrawGLScene();
  

  // Deactivate collision checking between the feet and the ground during planning
  mWorld->mCollisionHandle->getCollisionChecker()->deactivatePair(mWorld->getRobot(robotIndex)->getNode("leftFoot"), mWorld->getObject(groundIndex)->getNode(1));
  mWorld->mCollisionHandle->getCollisionChecker()->deactivatePair(mWorld->getRobot(robotIndex)->getNode("rightFoot"), mWorld->getObject(groundIndex)->getNode(1));
  
  Eigen::VectorXd kI = 100.0 * Eigen::VectorXd::Ones(mWorld->getRobot(robotIndex)->getNumDofs());
  Eigen::VectorXd kP = 500.0 * Eigen::VectorXd::Ones(mWorld->getRobot(robotIndex)->getNumDofs());
  Eigen::VectorXd kD = 100.0 * Eigen::VectorXd::Ones(mWorld->getRobot(robotIndex)->getNumDofs());
  std::vector<int> ankleDofs(2);
  ankleDofs[0] = 27;
  ankleDofs[1] = 28;
  const Eigen::VectorXd anklePGains = -1000.0 * Eigen::VectorXd::Ones(2);
  const Eigen::VectorXd ankleDGains = -2000.0 * Eigen::VectorXd::Ones(2);
  mController = new planning::Controller(mWorld->getRobot(robotIndex), actuatedDofs, kP, kD, ankleDofs, anklePGains, ankleDGains);
  planning::PathPlanner<> pathPlanner(*mWorld);

  std::list<Eigen::VectorXd> path;
  if(!pathPlanner.planPath(robotIndex, trajectoryDofs, mStartConf, mGoalConf, path)) {
    std::cout << "Path planner could not find a path" << endl;
  }
  else {
    const Eigen::VectorXd maxVelocity = 0.3 * Eigen::VectorXd::Ones(mRA_NumNodes);
    const Eigen::VectorXd maxAcceleration = 0.3 * Eigen::VectorXd::Ones(mRA_NumNodes);
    planning::Trajectory* trajectory = new planning::Trajectory(path, maxVelocity, maxAcceleration);
    std::cout << "Trajectory duration: " << trajectory->getDuration() << endl;
    mController->setTrajectory(trajectory, 0.1, trajectoryDofs);
  }
  
  mWorld->mCollisionHandle->getCollisionChecker()->activatePair(mWorld->getRobot(robotIndex)->getNode("leftFoot"), mWorld->getObject(groundIndex)->getNode(1));
  mWorld->mCollisionHandle->getCollisionChecker()->activatePair(mWorld->getRobot(robotIndex)->getNode("rightFoot"), mWorld->getObject(groundIndex)->getNode(1));
  
}

/**
 * @function simulate
 */
void pushDemoTab::simulate() {

  double totalTime = 6.0;
  int counter = 0;
 
  int numIter = (mDisplayTimeout / 1000.0) / mWorld->mTimeStep;

  while( mWorld->mTime < totalTime ) {

    counter++;
    printf("Counter: %d Current world time: %f num iter: %d\n", counter, mWorld->mTime, numIter );
    for (int i = 0; i < numIter; i++) {
      mWorld->getRobot(0)->setInternalForces(mController->getTorques(mWorld->getRobot(0)->getPose(), mWorld->getRobot(0)->getQDotVector(), mWorld->mTime));
      mWorld->step();
    }

    mSimFrame += numIter;

    bake();
    
    for (int j = 0; j < mWorld->getNumRobots(); j++) {
      mWorld->getRobot(j)->update();
    }
    for (int j = 0; j < mWorld->getNumObjects(); j++) {
      mWorld->getObject(j)->update();
      }
     viewer->DrawGLScene();
  }

  printf("Entered in loop %d times \n", counter);
  SetTimeline();
}

/**
 * @function SetTimeline
 */
void pushDemoTab::SetTimeline() {

  double T = 6.0;

  
  int numsteps = mBakedStates.size();

  double increment = T/(double)numsteps;

  cout << "-->(+) Updating Timeline - Increment: " << increment << " Total T: " << T << " Steps: " << numsteps << endl;

  frame->InitTimer( string("Planner"),increment );

  mPlayFrame = 0;
  for( int i = 0; i < numsteps; ++i ) {  
    retrieveBakedState( mPlayFrame );
    for (int j = 0; j < mWorld->getNumRobots(); j++) {
      mWorld->getRobot(j)->update();
    }
    for (int j = 0; j < mWorld->getNumObjects(); j++) {
      mWorld->getObject(j)->update();
    }
    frame->AddWorld( mWorld );
    mPlayFrame++;
  }
  printf("Finished setting timeline \n");
} 



/**
 * @function bake
 * @brief
 */
void pushDemoTab::bake() {

    VectorXd state(mWorld->mIndices.back());
    for(int i = 0; i < mWorld->getNumSkeletons(); i++) {
        state.segment(mWorld->mIndices[i], mWorld->mDofs[i].size()) = mWorld->mDofs[i];
    }
    mBakedStates.push_back(state);
}

/**
 * @function retrieveBakedState
 * @brief 
 */
void pushDemoTab::retrieveBakedState( int _frame ) {

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
void pushDemoTab::OnSlider(wxCommandEvent &evt) {

}


// This function is called when an object is selected in the Tree View or other
// global changes to the GRIP world. Use this to capture events from outside the tab.
void pushDemoTab::GRIPStateChange() {
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


