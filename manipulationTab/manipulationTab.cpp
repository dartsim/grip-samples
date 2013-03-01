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
#include <robotics/Object.h>
#include <robotics/Robot.h>
#include <planning/PathPlanner.h>
#include <planning/PathShortener.h>
#include <planning/Trajectory.h>
#include "Controller.h"
#include "Grasper.h"
// **********************

/** Events */
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
EVT_COMMAND(wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, manipulationTab::OnButton)
EVT_CHECKBOX(id_checkbox_showcollmesh, manipulationTab::OnCheckShowCollMesh)
END_EVENT_TABLE()
IMPLEMENT_DYNAMIC_CLASS(manipulationTab, GRIPTab)

/**
 * @function manipulationTab
 * @brief Constructor (TO BE USED WITH MANIPULATION)
 */
manipulationTab::manipulationTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, long style) :
GRIPTab(parent, id, pos, size, style) {
    sizerFull = new wxBoxSizer(wxHORIZONTAL);

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

void manipulationTab::OnCheckShowCollMesh(wxCommandEvent &evt) {
}

void manipulationTab::OnButton(wxCommandEvent & _evt) {

    int slnum = _evt.GetId();

    switch (slnum) {
            /** Set start configuration hard-coded (right arm) */
        case id_button_SetPredefStart: {
            mStartConf = mPredefStartConf;
        }
        break;
        case id_button_SetStart: {
            mStartConf = mWorld->getRobot(mRobotIndex)->getConfig(mArmDofs);
        }
        break;
           /** Show set start configuration */
        case id_button_ShowStart: {
            if(mStartConf.size()){
             cout << "Showing start conf for right arm: " << mStartConf.transpose() << endl;
             mWorld->getRobot(mRobotIndex)->setConfig(mArmDofs, mStartConf);
             viewer->DrawGLScene();
            }
            else{cout << "Must set start conf for right arm first!" << endl;}
        }
        break;
        case id_button_Grasping: {
            grasp();
        }
        break;
        case id_button_OpenHand: {
            if(grasper != NULL && palmEName.size()){
                grasper->openHand();
                viewer->DrawGLScene();
            }
            else{
                ECHO("ERROR: Must reinitialize Grasper object: Click Grasp Object!")
            }
        }
        break;
        case id_button_CloseHand: {
            if(grasper != NULL && palmEName.size()){
                //close with no target object; no collision checking peformed
                grasper->closeHand(0.1, NULL);
                viewer->DrawGLScene();
            }
            else{
                ECHO("ERROR: Must reinitialize Grasper object: Click Grasp Object!")
            }
        }
        break;
            /** Default */
        default: {
            printf("Default button \n");
        }
    }
}

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
    /*planning::PathShortener pathShortener(mWorld, mRobotIndex, mTotalDofs);
    pathShortener.shortenPath(path);*/
    
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
  mWorld->getRobot(mRobotIndex)->setInternalForces(mController->getTorques(mWorld->getRobot(mRobotIndex)->getPose(), mWorld->getRobot(mRobotIndex)->getQDotVector(), mWorld->mTime));
}

void manipulationTab::GRIPEventSimulationAfterTimestep() {
}

void manipulationTab::GRIPEventSimulationStart() {

}

void manipulationTab::GRIPStateChange() {
    if (selectedTreeNode == NULL) {
        return;
    }
    string statusBuf;
    string buf, buf2;
    switch (selectedTreeNode->dType) {
        case Return_Type_Object:{
            statusBuf = " Selected Object: ";
            buf = "You clicked on object: ";
            robotics::Object* pObject = (robotics::Object*)(selectedTreeNode->data);
            selectedNode = pObject->mRoot;
            break;
        }
        case Return_Type_Robot:{
            statusBuf = " Selected Robot: ";
            buf = "You clicked on robot: ";
            robotics::Robot* pRobot = (robotics::Robot*)(selectedTreeNode->data);
            selectedNode = pRobot->mRoot;
            break;
        }
        case Return_Type_Node:{
            statusBuf = " Selected Link:  of Robot: ";
            buf = " Link:  of Robot: ";
            dynamics::BodyNodeDynamics* pBodyNode = (dynamics::BodyNodeDynamics*)(selectedTreeNode->data);
            selectedNode = pBodyNode;
            break;
        }
        default:
            fprintf(stderr, "someone else's problem.");
            assert(0);
            exit(1);
    }
}

void manipulationTab::GRIPEventRender() {
    glDisable(GL_FOG);
    glEnable(GL_COLOR_MATERIAL);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    
    glLineWidth(1.5f);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POINT_SMOOTH);
    
    glActiveTexture(GL_TEXTURE0); 
    glEnable( GL_TEXTURE_2D );
    glTexEnvi(GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glBindTexture(GL_TEXTURE_2D, GL_TEXTURE_2D);
    
    mGroundIndex = 0;
   
     // draw collision meshes
    if (checkShowCollMesh->IsChecked() && mWorld && selectedNode && selectedNode->getShape()) {
        //ECHO("SHOWING MESHES");
        renderer::RenderInterface* ri = &viewer->renderer;
        kinematics::BodyNode* cnode = selectedNode;
        kinematics::Shape* shape = selectedNode->getShape();
        const aiScene* sc = shape->getVizMesh();
        if (shape->getCollisionMesh() != NULL) { sc = shape->getCollisionMesh(); }

        if (sc != NULL) {
            int verts = 0;
            const aiNode* nd = sc->mRootNode;
            
            //ECHO("SHAPE NOT NULL");
            // put in the proper transform
            glPushMatrix();
            double M[16];
            Eigen::Matrix4d worldTrans = selectedNode->getWorldTransform();
            for(int i=0;i<4;i++)
                for(int j=0;j<4;j++)
                    M[j*4+i] = worldTrans(i, j);
            glMultMatrixd(M);

            for (unsigned int n = 0; n < nd->mNumMeshes; ++n) {
                const struct aiMesh* mesh = sc->mMeshes[nd->mMeshes[n]];
                for (unsigned int t = 0; t < mesh->mNumFaces; ++t) {
                    const struct aiFace* face = &mesh->mFaces[t];
                    glBegin(GL_LINE_STRIP);
                    for(unsigned int i = 0; i < face->mNumIndices; i++) {
                        int index = face->mIndices[i];
                        glColor4d(0.0, 0.0, 1.0, 1.0);
                        if(mesh->mNormals != NULL) 
                            glNormal3fv(&mesh->mNormals[index].x);
                        glVertex3fv(&mesh->mVertices[index].x);
                        verts++;
                    }
                    glEnd();
                }
            }
            glPopMatrix();
        }

        glPopMatrix();
        glEnd();
    }
    
    //draw GCP and graspPoint
    if(checkShowCollMesh->IsChecked() && mWorld){        
        //draw axes origin = GCP
        //drawAxesWithOrientation(mWorld->getRobot(mRobotIndex)->getNode("GCP")->getWorldTransform(), 0.08);
        
        //draw axes origin = graspPoint; update everytime to move with object
        drawAxes(graspPoint, 0.08);
    }
    glFlush();
    
}

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

void manipulationTab::drawAxesWithOrientation(Eigen::Matrix4d transformation, double s){
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
// c-basic-offset: 2
// End:
