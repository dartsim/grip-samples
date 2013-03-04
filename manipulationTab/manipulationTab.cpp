/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

#include "manipulationTab.h"

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <iostream>

#include <collision/CollisionSkeleton.h>
#include <dynamics/SkeletonDynamics.h>
#include <dynamics/ContactDynamics.h>
#include <kinematics/ShapeBox.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
#include <planning/PathPlanner.h>
#include <planning/PathShortener.h>
#include <planning/Trajectory.h>
#include "Controller.h"
#include "Grasper.h"


/// Define IDs for buttons
enum DynamicSimulationTabEvents {
    id_button_DoPlanning = 8345,
    id_button_RelocateObjects,
    id_button_SetStart,
    id_button_SetGoal,
    id_button_SetPredefStart,
    id_button_SetPredefGoal,
    id_button_ShowStart,
    id_button_ShowGoal,
    id_button_Grasping,
    id_button_OpenHand,
    id_button_CloseHand,
    id_label_Inst,
    id_checkbox_showcollmesh
};
using namespace std;

// Handler for events
BEGIN_EVENT_TABLE(manipulationTab, wxPanel)
EVT_COMMAND(id_button_SetPredefStart, wxEVT_COMMAND_BUTTON_CLICKED, manipulationTab::onButtonSetPredefStart)
EVT_COMMAND(id_button_SetStart, wxEVT_COMMAND_BUTTON_CLICKED, manipulationTab::onButtonSetStart)
EVT_COMMAND(id_button_ShowStart, wxEVT_COMMAND_BUTTON_CLICKED, manipulationTab::onButtonShowStart)
EVT_COMMAND(id_button_Grasping, wxEVT_COMMAND_BUTTON_CLICKED, manipulationTab::onButtonDoGrasping)
EVT_COMMAND(id_button_OpenHand, wxEVT_COMMAND_BUTTON_CLICKED, manipulationTab::onButtonOpenHand)
EVT_COMMAND(id_button_CloseHand, wxEVT_COMMAND_BUTTON_CLICKED, manipulationTab::onButtonCloseHand)
EVT_CHECKBOX(id_checkbox_showcollmesh, manipulationTab::onCheckShowCollMesh)
END_EVENT_TABLE() 
IMPLEMENT_DYNAMIC_CLASS(manipulationTab, GRIPTab)


manipulationTab::manipulationTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, long style) :
GRIPTab(parent, id, pos, size, style) {
    wxSizer* sizerFull = new wxBoxSizer(wxHORIZONTAL);

    // Create Static boxes (outline of your Tab)
    wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("Setup"));
    wxStaticBox* ss2Box = new wxStaticBox(this, -1, wxT("Manipulation"));
    
    // Create sizers for these static boxes
    wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);
    wxStaticBoxSizer* ss2BoxS = new wxStaticBoxSizer(ss2Box, wxVERTICAL);
    
    // Start and goal conf button
    ss1BoxS->Add(new wxButton(this, id_button_SetPredefStart, wxT("Set Predef Start")), 0, wxALL, 1);
    ss1BoxS->Add(new wxButton(this, id_button_SetStart, wxT("Set Custom Start")), 0, wxALL, 1);
    ss1BoxS->Add(new wxButton(this, id_button_ShowStart, wxT("Show Start Conf")), 0, wxALL, 1);
    ss1BoxS->Add(new wxStaticText(this, id_label_Inst, wxT("Instructions:\n[1]Set start conf  [2]Select an object  [3]Click Grasp Object")
                                  ), 0, wxEXPAND);
    // Grasping
    ss2BoxS->Add(new wxButton(this, id_button_Grasping, wxT("Grasp Object")), 0, wxALL, 1);
    checkShowCollMesh = new wxCheckBox(this, id_checkbox_showcollmesh, wxT("Show Grasp Markers"));
    ss2BoxS->Add(checkShowCollMesh, 0, wxALL, 1);
   
    ss2BoxS->Add(new wxButton(this, id_button_OpenHand, wxT("Open Hand")), 0, wxEXPAND, 1);
    ss2BoxS->Add(new wxButton(this, id_button_CloseHand, wxT("Close Hand")), 0, wxEXPAND, 1);
    // Add the boxes to their respective sizers
    sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 6);
    sizerFull->Add(ss2BoxS, 1, wxEXPAND | wxALL, 6);
    SetSizer(sizerFull);

    // Additional settings
    mCurrentFrame = 0;
    mRobotIndex = 0; // We only simulate one robot in this demo so we know its index is 0

    mPredefStartConf.resize(6);
    mPredefStartConf << -0.858702, -0.674395, 0.0, -0.337896, 0.0, 0.0;
    mController = NULL;
}

/// Setup grasper when scene is loaded as well as populating arm's DoFs
void manipulationTab::GRIPEventSceneLoaded() {
  // Set initial configuration for the legs
  mWorld->getRobot(mRobotIndex)->getDof(19)->setValue(-10.0 * M_PI/180.0);
  mWorld->getRobot(mRobotIndex)->getDof(20)->setValue(-10.0 * M_PI/180.0);
  mWorld->getRobot(mRobotIndex)->getDof(23)->setValue(20.0 * M_PI/180.0);
  mWorld->getRobot(mRobotIndex)->getDof(24)->setValue(20.0 * M_PI/180.0);
  mWorld->getRobot(mRobotIndex)->getDof(27)->setValue(-10.0 * M_PI/180.0);
  mWorld->getRobot(mRobotIndex)->getDof(28)->setValue(-10.0 * M_PI/180.0);
  mWorld->getRobot(mRobotIndex)->update();
 
  // Define right arm nodes
  const string armNodes[] = {"Body_RSP", "Body_RSR", "Body_RSY", "Body_REP", "Body_RWY", "Body_RWP"};
  mArmDofs.resize(6);
  for(int i = 0; i < mArmDofs.size(); i++) {
    mArmDofs[i] = mWorld->getRobot(mRobotIndex)->getNode(armNodes[i].c_str())->getDof(0)->getSkelIndex();
  }
  
  //Define palm effector name; Note: this is robot dependent!
  palmEName = "Body_RWP";
  // Initialize Grasper; done here in order to allow Close and Open Hand buttons!
  grasper = new planning::Grasper(mWorld, mRobotIndex, palmEName);
  
}

/// Handle event for drawing grasp markers
void manipulationTab::onCheckShowCollMesh(wxCommandEvent &evt) {
}

/// Set start configuration to the configuration the arm is currently in
void manipulationTab::onButtonSetStart(wxCommandEvent& evt){
    if(!mWorld || mWorld->getNumRobots() < 1){
        cout << "No world loaded or world does not contain a robot" << endl;
        return;
    }
    mStartConf = mWorld->getRobot(mRobotIndex)->getConfig(mArmDofs);
    cout << "Start Configuration: " << mStartConf.transpose() << endl;
}

/// Reset start configuration to predefined one
void manipulationTab::onButtonSetPredefStart(wxCommandEvent& evt){
    if(!mWorld || mWorld->getNumRobots() < 1){
        cout << "No world loaded or world does not contain a robot" << endl;
        return;
    }
    mStartConf = mPredefStartConf;
}

/// Show the currently set start configuration
void manipulationTab::onButtonShowStart(wxCommandEvent& evt) {
    if (mStartConf.size()) {
        cout << "Showing start conf for right arm: " << mStartConf.transpose() << endl;
        mWorld->getRobot(mRobotIndex)->setConfig(mArmDofs, mStartConf);
        viewer->DrawGLScene();
    } else {
        ECHO("ERROR: Must set start conf for right arm first!");
    }
}

/// Test currently implemented grasping approach
void manipulationTab::onButtonDoGrasping(wxCommandEvent& evt){
    if(!mWorld || mWorld->getNumRobots() < 1){
        cout << "No world loaded or world does not contain a robot" << endl;
        return;
    }
    grasp();
}

/// Close robot's end effector
void manipulationTab::onButtonOpenHand(wxCommandEvent& evt) {
    if (grasper != NULL && palmEName.size()) {
        grasper->openHand();
        viewer->DrawGLScene();
    } else {
        ECHO("ERROR: Must reinitialize Grasper object: Click Grasp Object!")
    }
}

/// Open robot's end effector
void manipulationTab::onButtonCloseHand(wxCommandEvent& evt) {
    if (grasper != NULL && palmEName.size()) {
        grasper->closeHand(0.1, selectedNode);
        viewer->DrawGLScene();
    } else {
        ECHO("ERROR: Must reinitialize Grasper object: Click Grasp Object!")
    }
}

/// Set initial dynamic parameters and call grasp planner and controller
void manipulationTab::grasp() {
    
    if(selectedNode == NULL || mStartConf.size() == 0){ECHO("\tERROR: Must select an object to grasp first!!"); return;}
    // Perform memory management to allow for continuous grasping tests
    if(mController != NULL){
        delete mController;
        delete grasper;
        //re-init grasper
        grasper = new planning::Grasper(mWorld, mRobotIndex, palmEName);
    } 
    // Store the actuated joints (all except the first 6 which are only a convenience to locate the robot in the world)
    std::vector<int> actuatedDofs(mWorld->getRobot(mRobotIndex)->getNumDofs() - 6);
    for (unsigned int i = 0; i < actuatedDofs.size(); i++) {
        actuatedDofs[i] = i + 6;
    }
    
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
    const Eigen::VectorXd ankleDGains = -200.0 * Eigen::VectorXd::Ones(2);
    
    // Update robot's pose
    mWorld->getRobot(mRobotIndex)->setConfig(mArmDofs, mStartConf);
    
    // Create controller
    mController = new planning::Controller(mWorld->getRobot(mRobotIndex), actuatedDofs, kP, kD, ankleDofs, anklePGains, ankleDGains);
   
    // Setup grasper
    grasper->init(mArmDofs, mStartConf, selectedNode);
   
    // Perform grasp planning; now really it's just Jacobian translation
    std::list<Eigen::VectorXd> path;
    std::vector<int> mTotalDofs;
    grasper->plan(path, mTotalDofs);
    
    // CHECK
    PRINT(path.size());
    
    // Create trajectory; no need to shorten path here
    planning::PathShortener pathShortener(mWorld, mRobotIndex, mTotalDofs);
    pathShortener.shortenPath(path);
    
    const Eigen::VectorXd maxVelocity = 0.6 * Eigen::VectorXd::Ones(mTotalDofs.size());
    const Eigen::VectorXd maxAcceleration = 0.6 * Eigen::VectorXd::Ones(mTotalDofs.size());
    planning::Trajectory* trajectory = new planning::Trajectory(path, maxVelocity, maxAcceleration);
    
    std::cout << "-- Trajectory duration: " << trajectory->getDuration() << endl;
    mController->setTrajectory(trajectory, 0, mTotalDofs);
    
    // Reactivate collision of feet with floor Body_LAR Body_RAR
    mWorld->mCollisionHandle->getCollisionChecker()->activatePair(mWorld->getRobot(mRobotIndex)->getNode("Body_LAR"), ground->getNode(1));
    mWorld->mCollisionHandle->getCollisionChecker()->activatePair(mWorld->getRobot(mRobotIndex)->getNode("Body_RAR"), ground->getNode(1));

    printf("Controller time: %f \n", mWorld->mTime);
    
}

/// Before each simulation step we set the torques the controller applies to the joints
void manipulationTab::GRIPEventSimulationBeforeTimestep() {
    Eigen::VectorXd positionTorques = mController->getTorques(mWorld->getRobot(mRobotIndex)->getPose(), mWorld->getRobot(mRobotIndex)->getQDotVector(), mWorld->mTime);
    // section here to control the fingers for force-based grasping
    // instead of position-based grasping
    mWorld->getRobot(mRobotIndex)->setInternalForces(positionTorques);
}

/// Handle simulation events
void manipulationTab::GRIPEventSimulationAfterTimestep() {
}

/// Handle simulation events
void manipulationTab::GRIPEventSimulationStart() {

}

/// Store selected node in tree-view data as grasper's objective
void manipulationTab::GRIPStateChange() {
    if (!selectedTreeNode) {
        return;
    }
    switch (selectedTreeNode->dType) {
    case Return_Type_Object:
    case Return_Type_Robot:
        selectedNode = ((kinematics::Skeleton*)selectedTreeNode->data)->getRoot();
        break;
    case Return_Type_Node:
        selectedNode = (kinematics::BodyNode*)selectedTreeNode->data;
        break;
    default:
        fprintf(stderr, "someone else's problem.");
        assert(0);
        exit(1);
    }
}

/// Render grasp' markers such as grasping point and GCP(later)
void manipulationTab::GRIPEventRender() {
    mGroundIndex = 0;
    //draw graspPoint
    if(checkShowCollMesh->IsChecked() && mWorld){        
        //draw axes origin = graspPoint; update everytime to move with object
        drawAxes(graspPoint, 0.08);
    }
    glFlush();
}

/// Method to draw XYZ axes
void manipulationTab::drawAxes(Eigen::VectorXd origin, double s){
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);
    glVertex3f(origin(0) - s, origin(1), origin(2));
    glVertex3f(origin(0) + s, origin(1), origin(2));

    glColor3f(0, 0, 1);
    glVertex3f(origin(0), origin(1) - s, origin(2));
    glVertex3f(origin(0), origin(1) + s, origin(2));

    glColor3f(0, 1, 0);
    glVertex3f(origin(0), origin(1), origin(2) - s);
    glVertex3f(origin(0), origin(1), origin(2) + s);
    glEnd();
}

/// Method to draw XYZ axes with proper orientation. Collaboration with Justin Smith
void manipulationTab::drawAxesWithOrientation(const Eigen::Matrix4d& transformation, double s){
    Eigen::Matrix4d basis1up, basis1down, basis2up, basis2down;
    basis1up << s,  0.0, 0.0, 0,
     				0.0, s,   0.0, 0,
     				0.0, 0.0, s,   0,
     				1.0, 1.0, 1.0, 1;
     				
    basis1down << -s,  0.0, 0.0, 0,
     				0.0, -s,   0.0, 0,
     				0.0, 0.0, -s,   0,
     				1.0, 1.0, 1.0, 1;
    
    basis2up = transformation * basis1up;
    basis2down = transformation * basis1down;
    
    
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);
    glVertex3f(basis2down(0,0), basis2down(1,0), basis2down(2,0));
    glVertex3f(basis2up(0,0), basis2up(1,0), basis2up(2,0));

    glColor3f(0, 0, 1);
    glVertex3f(basis2down(0,1), basis2down(1,1), basis2down(2,1));
    glVertex3f(basis2up(0,1), basis2up(1,1), basis2up(2,1));

    glColor3f(0, 1, 0);
    glVertex3f(basis2down(0,2), basis2down(1,2), basis2down(2,2));
    glVertex3f(basis2up(0,2), basis2up(1,2), basis2up(2,2));
    glEnd();
}

// Local Variables:
// c-basic-offset: 4
// End:
