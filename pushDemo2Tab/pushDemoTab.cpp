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
#include <kinematics/ShapeBox.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
#include <robotics/Object.h>
#include <robotics/Robot.h>

// Planning and controller
#include <planning/PathPlanner.h>
#include <planning/PathShortener.h>
#include <planning/Trajectory.h>
#include "Controller.h"
// **********************


/** Events */
enum DynamicSimulationTabEvents {
  id_button_AddFloor = 8345,
  id_button_DoPlanning,
  id_button_RelocateObjects,
  id_button_SetStart,
  id_button_SetGoal,
  id_button_SetPredefStart,
  id_button_SetPredefGoal,
  id_button_ShowStart,
  id_button_ShowGoal,
  id_button_SetTimeline
};

/** Handler for events **/
BEGIN_EVENT_TABLE(pushDemoTab, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, pushDemoTab::OnButton)
EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, pushDemoTab::OnSlider)
END_EVENT_TABLE()


// Class constructor for the tab: Each tab will be a subclass of GRIPTab
IMPLEMENT_DYNAMIC_CLASS(pushDemoTab, GRIPTab)

// Define right arm nodes
string const pushDemoTab::mRA_Nodes[mRA_NumNodes] = {"Body_RSP", "Body_RSR", "Body_RSY", "Body_REP", "Body_RWY", "rightUJoint", "rightPalmDummy"}; 

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
  ss1BoxS->Add(new wxButton(this, id_button_SetGoal, wxT("Set Goal Conf")), 0, wxALL, 1); 
  ss1BoxS->Add(new wxButton(this, id_button_SetPredefStart, wxT("Set Predef Start")), 0, wxALL, 1); 
  ss1BoxS->Add(new wxButton(this, id_button_SetPredefGoal, wxT("Set Predef Goal")), 0, wxALL, 1); 

  // Check buttons (visualize the start and goal states)
  ss2BoxS->Add(new wxButton(this, id_button_ShowStart, wxT("Show Start")), 0, wxALL, 1); 
  ss2BoxS->Add(new wxButton(this, id_button_ShowGoal, wxT("Show Goal")), 0, wxALL, 1); 

  // Dynamic configuration buttons
  ss3BoxS->Add(new wxButton(this, id_button_AddFloor, wxT("Add floor")), 0, wxALL, 1); 
  ss3BoxS->Add(new wxButton(this, id_button_DoPlanning, wxT("Do Planning")), 0, wxALL, 1); 
  ss3BoxS->Add(new wxButton(this, id_button_RelocateObjects, wxT("Relocate objects")), 0, wxALL, 1); 
  ss3BoxS->Add(new wxButton(this, id_button_SetTimeline, wxT("Set Timeline")), 0, wxALL, 1); 
  
  // Add the boxes to their respective sizers
  sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 6);
  sizerFull->Add(ss2BoxS, 1, wxEXPAND | wxALL, 6);
  sizerFull->Add(ss3BoxS, 1, wxEXPAND | wxALL, 6);

  SetSizer(sizerFull);

  // Additional settings
  mCurrentFrame = 0;

  mRobotIndex = 0; // We only simulate one robot in this demo so we know its index is 0
  mGroundIndex = -1; // Default if not created

  mPredefStartConf.resize( mRA_NumNodes );
  mPredefGoalConf.resize( mRA_NumNodes );
  
  
  // Start and Conf with furniture_2
  mPredefStartConf << -0.858702, -0.674395, 0, -0.337896, 0, 0, 0;
  mPredefGoalConf << -0.69115, 0.121475, 0.284977, -1.02486, 0, 0, 0;


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


    // Set Goal Arm Configuration
  case id_button_SetGoal: {

    if( mWorld != NULL ) { 
      if( mWorld->getNumRobots() < 1 ) {  
	printf("No robot in the loaded world, you need one! \n"); break; 
      }
                       
      std::cout<<"* Goal Conf: ";
      mGoalConf.resize( mRA_NumNodes );
      for(int i = 0; i < mRA_NumNodes; i++) {
	mGoalConf[i] = mWorld->getRobot(mRobotIndex)->getNode( mRA_Nodes[i].c_str() )->getDof(0)->getValue();
	std::cout<< " "<<mGoalConf[i];
      }  
      std::cout<<std::endl;
    }      
    else {  
      std::cout<<"--(!) No world loaded, I cannot set a start position (!)--\n"<<std::endl;
    }

  }
    break;

    /** DoPlanning*/
  case id_button_DoPlanning: {
    initSettings();
  } break;

    /** Relocate objects */
  case id_button_RelocateObjects: {
    relocateObjects();
  } 
    break;

    /** Set start configuration hard-coded (right arm) */
  case id_button_SetPredefStart : {
    mStartConf.resize( mRA_NumNodes );
    mStartConf = mPredefStartConf;

  } break;

    /** Set goal configuration hard-coded (right arm) */
  case id_button_SetPredefGoal : {
    mGoalConf.resize( mRA_NumNodes );
    mGoalConf = mPredefGoalConf;
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
    if( mGoalConf.size() < 1 ) {  
      std::cout<<"You have not set a Goal configuration yet"<<std::endl; 
      break; 
    }   

    // Setting goal configuration
    std::cout<< "Showing goal conf for right arm: ";
    for(int i = 0; i < mRA_NumNodes; i++) {
      mWorld->getRobot(mRobotIndex)->getNode( mRA_Nodes[i].c_str() )->getDof(0)->setValue( mGoalConf(i) );
      std::cout<<" "<<mGoalConf(i)<<" ";
    }  
    std::cout<<std::endl;

    mWorld->getRobot(mRobotIndex)->update();
    viewer->DrawGLScene();
  } break;

    /** Set Timeline */
  case id_button_SetTimeline : {
    setTimeline();
  } break;
      
    /** Default */
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
  robotics::Object* ground = new robotics::Object();
  ground->setName("ground");
  ground->addDefaultRootNode();
  dynamics::BodyNodeDynamics* node = new dynamics::BodyNodeDynamics();
  node->setShape( new kinematics::ShapeBox( Eigen::Vector3d( 10.0, 10.0, 0.0001), 1.0));

  kinematics::Joint* joint = new kinematics::Joint( ground->getRoot(), node );
  ground->addNode( node );
  ground->initSkel();
  ground->update();
  ground->setImmobileState( true );
  mWorld->addObject( ground );
  mWorld->rebuildCollision();

  treeView->CreateFromWorld();


  // Get the ground object index
  mGroundIndex = -1;
  for( int i = 0; i < mWorld->getNumObjects(); ++i ) {
    if( mWorld->getObject(i)->getName() == "ground" ) {
      mGroundIndex = i; break;
    }
  }

  if( mGroundIndex == -1 ) { printf("I did not find the floor! EXITING \n"); return; }
  else { printf("-- Ground is object %d \n", mGroundIndex);}
    
  printf("-- Added floor \n");
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
    printf(" %f ", trajectoryDofs[i]);
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
  mWorld->mCollisionHandle->getCollisionChecker()->deactivatePair(mWorld->getRobot(mRobotIndex)->getNode("leftFoot"), mWorld->getObject(mGroundIndex)->getNode(1));
  mWorld->mCollisionHandle->getCollisionChecker()->deactivatePair(mWorld->getRobot(mRobotIndex)->getNode("rightFoot"), mWorld->getObject(mGroundIndex)->getNode(1));
  
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

  // Create Planner
  planning::PathPlanner<> pathPlanner(*mWorld);


  // Call Planner
  std::list<Eigen::VectorXd> path;
  if(!pathPlanner.planPath(mRobotIndex, trajectoryDofs, mStartConf, mGoalConf, path)) {
    std::cout << "<!> Path planner could not find a path" << std::endl;
  }
  else {
    planning::PathShortener pathShortener(mWorld, mRobotIndex, trajectoryDofs);
    pathShortener.shortenPath(path);

    mWorld->getRobot(mRobotIndex)->update();

    const Eigen::VectorXd maxVelocity = 0.6 * Eigen::VectorXd::Ones(mRA_NumNodes);
    const Eigen::VectorXd maxAcceleration = 0.6 * Eigen::VectorXd::Ones(mRA_NumNodes);
    planning::Trajectory* trajectory = new planning::Trajectory(path, maxVelocity, maxAcceleration);
    std::cout << "-- Trajectory duration: " << trajectory->getDuration() << endl;
    //mController->setTrajectory(trajectory, 0.1, trajectoryDofs);
    mController->setTrajectory(trajectory, 0, trajectoryDofs);
  }

  
  // Reactivate collision of feet with floor
  mWorld->mCollisionHandle->getCollisionChecker()->activatePair(mWorld->getRobot(mRobotIndex)->getNode("leftFoot"), mWorld->getObject(mGroundIndex)->getNode(1));
  mWorld->mCollisionHandle->getCollisionChecker()->activatePair(mWorld->getRobot(mRobotIndex)->getNode("rightFoot"), mWorld->getObject(mGroundIndex)->getNode(1));
  printf("Controller time: %f \n", mWorld->mTime);
}

/**
 * @function relocateObjects
 */
void pushDemoTab::relocateObjects() {

  int orangeCubeIndex = -1;
  int yellowCubeIndex = -1;

  for( int i = 0; i < mWorld->getNumObjects(); ++i ) {
    if( mWorld->getObject(i)->getName() == "orangeCube" ) {
      orangeCubeIndex = i; break;
    }
  }

  for( int i = 0; i < mWorld->getNumObjects(); ++i ) {
    if( mWorld->getObject(i)->getName() == "yellowCube" ) {
      yellowCubeIndex = i; break;
    }
  }

  if( orangeCubeIndex == -1 || yellowCubeIndex == -1 ) { printf("Did not find orange or yellow object. Exiting and no moving anything \n"); return; }
  
  // Set positions
  mWorld->getObject(orangeCubeIndex)->setPositionXYZ( 0.30, -0.30, 0.83 );
  mWorld->getObject(yellowCubeIndex)->setPositionXYZ( 0.30, -0.30, 0.935 );

  // Update
  mWorld->getObject(orangeCubeIndex)->update();
  mWorld->getObject(yellowCubeIndex)->update();

  // Draw nicely
  viewer->DrawGLScene();
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

  mCurrentFrame++;
  if( mCurrentFrame % 30 == 0 ) {
    bake();
  }
}

/**
 * @function GRIPEventSimulationStart
 * @brief
 */
void pushDemoTab::GRIPEventSimulationStart() {

}

/**
 * @function setTimeline
 * @brief Store the simulated poses in the Timeline slider for playback
 */
void pushDemoTab::setTimeline() {

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
 * @brief Store a world state at some step
 */
void pushDemoTab::bake() {
    mBakedStates.push_back(mWorld->getState());
}

/**
 * @function retrieveBakedState
 * @brief Return a vector with the poses stored at frame _frame
 */
void pushDemoTab::retrieveBakedState( int _frame ) {
    mWorld->setState(mBakedStates[_frame]);
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


