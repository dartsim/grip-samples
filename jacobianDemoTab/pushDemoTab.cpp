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
#include "JTFollower/JTFollower.h"

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
#include <kinematics/ShapeBox.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
#include <robotics/Object.h>
#include <robotics/Robot.h>

// Planning and controller
#include <planning/PathPlanner.h>
#include <planning/Trajectory.h>
#include <planning/PathShortener.h>
#include "Controller.h"
// **********************


/** Events */
enum DynamicSimulationTabEvents {
  id_button_DoPlanning = 8345,
  id_button_SetStart,
  id_button_SetGoal,
  id_button_SetPredefStart,
  id_button_ShowStart,
  id_button_ShowGoal,
};

/** Handler for events **/
BEGIN_EVENT_TABLE(pushDemoTab, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, pushDemoTab::OnButton)
EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, pushDemoTab::OnSlider)
END_EVENT_TABLE()


// Class constructor for the tab: Each tab will be a subclass of GRIPTab
IMPLEMENT_DYNAMIC_CLASS(pushDemoTab, GRIPTab)

// Define right arm nodes
string const pushDemoTab::mRA_Nodes[mRA_NumNodes] = {"Body_RSP", "Body_RSR", "Body_RSY", "Body_REP", "Body_RWY", "Body_RWP"}; 

/**
 * @function pushDemoTab
 * @brief Constructor (TO BE USED WITH FURNITURE_2)
 */
pushDemoTab::pushDemoTab(wxWindow *parent, const wxWindowID id,
		const wxPoint& pos, const wxSize& size, long style) :
  GRIPTab(parent, id, pos, size, style) {
  sizerFull = new wxBoxSizer(wxHORIZONTAL);
  
  // Create Static boxes (outline of your Tab)
  wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("Planning"));
  wxStaticBox* ss2Box = new wxStaticBox(this, -1, wxT("Check"));
  wxStaticBox* ss3Box = new wxStaticBox(this, -1, wxT("Dynamic Settings"));
  
  // Create sizers for these static boxes
  wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);
  wxStaticBoxSizer* ss2BoxS = new wxStaticBoxSizer(ss2Box, wxVERTICAL);
  wxStaticBoxSizer* ss3BoxS = new wxStaticBoxSizer(ss3Box, wxVERTICAL);

  // Start and goal conf buttons
  ss1BoxS->Add(new wxButton(this, id_button_SetStart, wxT("Set Start Conf")), 0, wxALL, 1); 
  ss1BoxS->Add(new wxButton(this, id_button_SetGoal, wxT("Set Goal Object")), 0, wxALL, 1); 
  ss1BoxS->Add(new wxButton(this, id_button_SetPredefStart, wxT("Set Predef Start")), 0, wxALL, 1); 

  // Check buttons (visualize the start and goal states)
  ss2BoxS->Add(new wxButton(this, id_button_ShowStart, wxT("Show Start")), 0, wxALL, 1); 
  ss2BoxS->Add(new wxButton(this, id_button_ShowGoal, wxT("Show Goal")), 0, wxALL, 1); 

  // Dynamic configuration buttons
  ss3BoxS->Add(new wxButton(this, id_button_DoPlanning, wxT("Do Planning")), 0, wxALL, 1);  
  
  // Add the boxes to their respective sizers
  sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 6);
  sizerFull->Add(ss2BoxS, 1, wxEXPAND | wxALL, 6);
  sizerFull->Add(ss3BoxS, 1, wxEXPAND | wxALL, 6);

  SetSizer(sizerFull);

  // Additional settings
  mCurrentFrame = 0;

  mRobotIndex = 0; // We only simulate one robot in this demo so we know its index is 0

  mPredefStartConf.resize( mRA_NumNodes );
  
  // Start and Conf with furniture_2
  mPredefStartConf <<  -1.20687,  -1.11899,  0,  0,  0,  0 ;
  mGoalObject = "smallRedBall";

}


/**
 * @function OnButton
 * @brief Handles button events
 */
void pushDemoTab::OnButton(wxCommandEvent & _evt) {
  
  int slnum = _evt.GetId();
  
  switch( slnum ) {
  
    // Set Start Arm Configuration
  case id_button_SetStart: {
    
    if( mWorld != NULL ) { 
      if( mWorld->getNumRobots() < 1 ) {  
	printf("No robot in the loaded world, you need one! \n"); break; 
      }
      
      std::cout<<"* Start Conf: ";
      mStartConf.resize( mRA_NumNodes );
      for(int i = 0; i < mRA_NumNodes; i++) {
	mStartConf[i] = mWorld->getRobot(mRobotIndex)->getNode( mRA_Nodes[i].c_str() )->getDof(0)->getValue();
	std::cout<< " "<<mStartConf[i];
      }  
      std::cout<<std::endl;
    }      
    else {  
      std::cout<<"--(!) No world loaded, I cannot set a start position (!)--\n"<<std::endl;
    }
      
  }
    break;


    // Set Goal Location
  case id_button_SetGoal: {

    if( mWorld != NULL ) { 
      if( mWorld->getNumRobots() < 1 ) {  
	printf("No robot in the loaded world, you need one! \n"); break; 
      }
       
      // Store the selected treeViewer element
      mGoalObjectIndex = -1;
      for( int i = 0; i < mWorld->getNumObjects(); ++i ) {
	if( mWorld->getObject(i)->getName() == mGoalObject ) {
	  mGoalObjectIndex = i; break;
	}
      }
       
      if( mGoalObjectIndex == -1 ) {
	std::cout<< "* Select an object from the Tree View! No goal position set!" <<std::endl;
      }
      else {
	mGoalPos.resize( mSizePos );
	mWorld->getObject( mGoalObjectIndex )->getPositionXYZ( mGoalPos(0), mGoalPos(1), mGoalPos(2) );
	
	std::cout<<"* Goal object: "<<mGoalObject<<" with index: "<<mGoalObjectIndex<<std::endl;
	std::cout<<"* Goal Pos: "<< mGoalPos(0)<<" , "<< mGoalPos(1) << " , "<< mGoalPos(2)<<std::endl;
      }
    }      
    else {  
      std::cout<<"--(!) No world loaded, I cannot set a goal position (!)--\n"<<std::endl;
    }
    
  }
    break;
    
    /** DoPlanning*/
  case id_button_DoPlanning: {
    initSettings();
  } break;


    /** Set start configuration hard-coded (right arm) */
  case id_button_SetPredefStart : {
    mStartConf.resize( mRA_NumNodes );
    mStartConf = mPredefStartConf;
    
  } break;


    /** Show set start configuration */
  case id_button_ShowStart : {
    if( mStartConf.size() < 1 ) {  
      std::cout<<"You have not set a Start configuration yet"<<std::endl; 
      break; 
    }   

    // Setting start configuration
    std::cout<< "Showing start conf for right arm: ";
    for(int i = 0; i < mRA_NumNodes; i++) {
      mWorld->getRobot(mRobotIndex)->getNode( mRA_Nodes[i].c_str() )->getDof(0)->setValue( mStartConf(i) );
      std::cout<<" "<<mStartConf(i)<<" ";
    }  
    std::cout<<std::endl;

    mWorld->getRobot(mRobotIndex)->update();
    viewer->DrawGLScene();
  } break;

    /** Show set goal configuration */
  case id_button_ShowGoal : {
    if( mGoalPos.size() < 1 ) {  
      std::cout<<"You have not set a Goal position yet"<<std::endl; 
      break; 
    }   

    // Setting goal configuration
    std::cout<< "Showing goal conf for right arm: ";
    for(int i = 0; i < mSizePos; i++) {
      std::cout<<" "<<mGoalPos(i);
    }  
    std::cout<<std::endl;
    
  } break;

    /** Default */
  default: {
    printf("Default button \n");
    }
  }
}


/**
 * @function initSettings
 * @brief Set initial dynamic parameters and call planner and controller
 */
void pushDemoTab::initSettings() {

  // Get the indices of the right arm's DOF
  std::vector<int> trajectoryDofs( mRA_NumNodes);
  printf("Trajectory nodes are: ");
  for(int i = 0; i < mRA_NumNodes; i++) {
    trajectoryDofs[i] = mWorld->getRobot(mRobotIndex)->getNode(mRA_Nodes[i].c_str())->getDof(0)->getSkelIndex();
    printf(" %d ", trajectoryDofs[i]);
  }
  printf("\n");
  
  // Store the actuated joints (all except the first 6 which are only a convenience to locate the robot in the world)
  std::vector<int> actuatedDofs(mWorld->getRobot(mRobotIndex)->getNumDofs() - 6);
  for(unsigned int i = 0; i < actuatedDofs.size(); i++) {
    actuatedDofs[i] = i + 6;
  }
  
  // Set initial configuration for the legs
  mWorld->getRobot(mRobotIndex)->getDof(19)->setValue(-10.0 * M_PI/180.0);
  mWorld->getRobot(mRobotIndex)->getDof(20)->setValue(-10.0 * M_PI/180.0);
  mWorld->getRobot(mRobotIndex)->getDof(23)->setValue(20.0 * M_PI/180.0);
  mWorld->getRobot(mRobotIndex)->getDof(24)->setValue(20.0 * M_PI/180.0);
  mWorld->getRobot(mRobotIndex)->getDof(27)->setValue(-10.0 * M_PI/180.0);
  mWorld->getRobot(mRobotIndex)->getDof(28)->setValue(-10.0 * M_PI/180.0);

  // Set initial configuration for the right arm
  for(int i = 0; i < mRA_NumNodes; i++) {
    mWorld->getRobot(mRobotIndex)->getNode( mRA_Nodes[i].c_str() )->getDof(0)->setValue( mStartConf(i) );
  }  
  
  // Update the view
  mWorld->getRobot(mRobotIndex)->update();
  viewer->DrawGLScene();
  
  printf("Set initial configuration for the legs \n");

  // Deactivate collision checking between the feet and the ground during planning
  dynamics::SkeletonDynamics* ground = mWorld->getSkeleton("ground");
  mWorld->mCollisionHandle->getCollisionChecker()->deactivatePair(mWorld->getRobot(mRobotIndex)->getNode("Body_LAR"), ground->getNode(1));
  mWorld->mCollisionHandle->getCollisionChecker()->deactivatePair(mWorld->getRobot(mRobotIndex)->getNode("Body_RAR"), ground->getNode(1));
  
  // Define PD controller gains
  Eigen::VectorXd kI = 100.0 * Eigen::VectorXd::Ones(mWorld->getRobot(mRobotIndex)->getNumDofs());
  Eigen::VectorXd kP = 500.0 * Eigen::VectorXd::Ones(mWorld->getRobot(mRobotIndex)->getNumDofs());
  Eigen::VectorXd kD = 100.0 * Eigen::VectorXd::Ones(mWorld->getRobot(mRobotIndex)->getNumDofs());

  // Define gains for the ankle PD
  std::vector<int> ankleDofs(2);
  ankleDofs[0] = 27;
  ankleDofs[1] = 28;
  const Eigen::VectorXd anklePGains = -1000.0 * Eigen::VectorXd::Ones(2);
  const Eigen::VectorXd ankleDGains = -2000.0 * Eigen::VectorXd::Ones(2);

  // Create controller
  mController = new planning::Controller(mWorld->getRobot(mRobotIndex), actuatedDofs, kP, kD, ankleDofs, anklePGains, ankleDGains);

  // Find path
  Eigen::VectorXd initState = mWorld->getState();
  std::list<Eigen::VectorXd> path;
  path = getPath();
  planning::PathShortener pathShortener(mWorld, mRobotIndex, trajectoryDofs);
  pathShortener.shortenPath(path);
  mWorld->setState(initState);

  if( path.size() < 1 ) {
    std::cout << "<!> Path planner could not find a path" << std::endl;
  }
  else {

  mWorld->getRobot(mRobotIndex)->update();

  const Eigen::VectorXd maxVelocity = 0.3 * Eigen::VectorXd::Ones(mRA_NumNodes);
  const Eigen::VectorXd maxAcceleration = 0.3 * Eigen::VectorXd::Ones(mRA_NumNodes);
  planning::Trajectory* trajectory = new planning::Trajectory(path, maxVelocity, maxAcceleration);
  std::cout << "-- Trajectory duration: " << trajectory->getDuration() << endl;
  //mController->setTrajectory(trajectory, 0.1, trajectoryDofs);
  mController->setTrajectory(trajectory, 0, trajectoryDofs);
  }
  
  // Reactivate collision of feet with floor
  mWorld->mCollisionHandle->getCollisionChecker()->activatePair(mWorld->getRobot(mRobotIndex)->getNode("Body_LAR"), ground->getNode(1));
  mWorld->mCollisionHandle->getCollisionChecker()->activatePair(mWorld->getRobot(mRobotIndex)->getNode("Body_RAR"), ground->getNode(1));
  printf("Controller time: %f \n", mWorld->mTime);
}

/**
 * @function getPathToPos
 */
std::list<Eigen::VectorXd> pushDemoTab::getPath() {

  printf("Running JT Follower \n");
  std::list<Eigen::VectorXd> path;

  std::vector<int> trajectoryDofs;
  trajectoryDofs.resize( mRA_NumNodes );

  printf("Trajectory nodes are: \n");
  for(int i = 0; i < mRA_NumNodes; i++) {
    trajectoryDofs[i] = mWorld->getRobot(mRobotIndex)->getNode(mRA_Nodes[i].c_str())->getDof(0)->getSkelIndex();
    printf(" %d ", trajectoryDofs[i]);
  }
  printf("\n");


  JTFollower *jt = new JTFollower(*mWorld);

  mEEName = "Body_RWP";
  for( int i = 0; i < mWorld->getRobot( mRobotIndex )->getNumNodes(); ++i ) {\
    if( mWorld->getRobot( mRobotIndex )->getNode(i)->getName() == mEEName ) {
      mEEId = i; break;
    }
  }

  jt->init( mRobotIndex, trajectoryDofs, mEEName, mEEId, 0.02 ); 
  
  std::vector<Eigen::VectorXd> wsPath; 
  Eigen::VectorXd start = mStartConf; 
  if( jt->GoToXYZ( start,  
		   mGoalPos,  
		   wsPath ) == true ) { 
    printf("Found solution JT! \n"); 
  } 
  else{ 
    printf("NO Found solution JT! Plotting anyway \n"); 
  }   

  // Save to list to generate trajectory
  for( int i = 0; i < wsPath.size(); ++i ) {
    path.push_back( wsPath[i] );
  }
  return path;

}


/**
 * @function GRIPEventSimulationBeforeTimeStep
 * @brief Before each sim step we must set the internal forces 
 */
void pushDemoTab::GRIPEventSimulationBeforeTimestep() {

  mWorld->getRobot(mRobotIndex)->setInternalForces(mController->getTorques(mWorld->getRobot(mRobotIndex)->getPose(), mWorld->getRobot(mRobotIndex)->getQDotVector(), mWorld->mTime));
 
}

/**
 * @function GRIPEventSimulationAfterTimeStep
 * @brief After 30 sim steps we save frames for future playback
 */
void pushDemoTab::GRIPEventSimulationAfterTimestep() {
}

/**
 * @function GRIPEventSimulationStart
 * @brief
 */
void pushDemoTab::GRIPEventSimulationStart() {

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
    mGoalObject = ( (robotics::Object*)(selectedTreeNode->data) )->getName();
    
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


