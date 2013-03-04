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

/** @file Grasper.cpp
 *  @author Juan C. Garcia
 */

#include "Grasper.h"
#include "robotics/World.h"
#include "robotics/Robot.h"
#include "kinematics/Dof.h"
#include <GUI/Viewer.h>
#include <Eigen/LU>
#include <set>
#include <dart/dynamics/ContactDynamics.h>
#include <dart/collision/CollisionSkeleton.h>
#include "manipulationTab.h"

using namespace std;
using namespace Eigen;
using namespace robotics;
using namespace collision_checking;

//Marker for OpenGL drawing
Vector3d graspPoint;

namespace planning {
    Grasper::Grasper(World* w, int r, string mEEName) {
        world = w;
        robot = r;
        EEName = mEEName;
    }
    
    Grasper::~Grasper() {
        delete jm;
    }
    
    /// Finish initialization of grasper by providing end effector's links ids, start configuration and target object
    void Grasper::init(std::vector<int>& d, Eigen::VectorXd& start, kinematics::BodyNode* node){
        dofs = d; 
        startConfig = start;
        objectNode = node;
        
        //initialize JointMover with end effector and arm's DoFs
        jm = new JointMover(*world, robot, dofs, EEName, 0.01);
    }
    
    /// Attempt a grasp at a target object
    void Grasper::plan(list<VectorXd> &path, vector<int> &totalDofs) {
        //find closest point in target object; grasp target point
        int min = calculateMinDistance(graspPoint); 
        VectorXd goalPose(6);
        
        //perform translation Jacobian towards grasping point computed
        jm->GoToXYZ(startConfig, graspPoint, goalPose, path);
        
        //try to close hand;  //QUICK FIX: expand each vector in path to be 21 DOFs
        totalDofs = dofs;
        closeHand(0.1, objectNode);
        PRINT(hand_dofs.size());
        //merge DOFS
        totalDofs.insert(totalDofs.end(), hand_dofs.begin(), hand_dofs.end());
        //increase size of every vector in path
        for(list<VectorXd>::iterator it = path.begin(); it != path.end(); it++){
            VectorXd & v (*it);
            v.conservativeResize(totalDofs.size()); v.segment(dofs.size(),hand_dofs.size()) = VectorXd::Zero(hand_dofs.size(), 1);
        }
        path.push_back(world->getRobot(robot)->getConfig(totalDofs));
        ECHO("Added closed hand config to path!");
        //PRINT(world->getRobot(robot)->getConfig(totalDofs));
        //move grasped object around
        list<VectorXd> targetPoints; 
        VectorXd v(3); v << 0.33,-0.10, 1.17; targetPoints.push_back(v);
        VectorXd w(3); w << 0.33,-0.12, 1.15; targetPoints.push_back(w);
        VectorXd u(3); u << 0.33,-0.6, 1.14; targetPoints.push_back(u);
        VectorXd backPose(6);
        list<VectorXd> path_back;
        // move to 3 target points and store paths
        for(list<VectorXd>::iterator loc = targetPoints.begin(); loc != targetPoints.end(); loc++){
            VectorXd & t(*loc);
            jm->GoToXYZ(world->getRobot(robot)->getConfig(dofs), t, backPose, path_back);
            for(list<VectorXd>::iterator it = path_back.begin(); it != path_back.end(); it++){
                    VectorXd & v (*it);
                    v.conservativeResize(totalDofs.size()); v.segment(dofs.size(),hand_dofs.size()) = world->getRobot(robot)->getConfig(hand_dofs);
                    //merge lists
                    path.push_back(v);
            }
        }
        //open hand at the end and store such configuration
        openHand();
        path.push_back(world->getRobot(robot)->getConfig(totalDofs));
    }
    
    /// Find closest point in target object to be grasped
    double Grasper::calculateMinDistance(Vector3d &closest){
        //1. get collision meshes and vertices
    	kinematics::ShapeMesh* shapeMesh = dynamic_cast<kinematics::ShapeMesh *>(objectNode->getColShape());
        
    	if(shapeMesh == NULL){
    		return -1;
        }
        const aiScene* sc = shapeMesh->getMesh();

        double min_distance = -1;
         
        if(sc != NULL){
            const aiNode* nd = sc->mRootNode;
            Matrix4d worldTrans = objectNode->getWorldTransform();
            const struct aiMesh* mesh = sc->mMeshes[nd->mMeshes[0]];
           
            //2. find closest vertex
            for(int j=0; j<mesh->mNumVertices; j++){
                
                VectorXd vertices(4); vertices <<  mesh->mVertices[j].x, mesh->mVertices[j].y , mesh->mVertices[j].z, 1;
                vertices = worldTrans * vertices; //now vertices turned into 4x1 matrix

                //calculate distance between current vertex and GCP
                Vector3d diff; diff << vertices(0,0), vertices(1,0), vertices(2,0);
                Vector3d GCP;
                GCP = world->getRobot(robot)->getNode(EEName.c_str())->getWorldCOM();
                diff = diff - GCP;
                
                if(min_distance == -1 || diff.norm() < min_distance){
                    min_distance = diff.norm();
                    closest << vertices(0,0), vertices(1,0), vertices(2,0);
                }    
            }
        }
        return min_distance;
    }
    
    /// Modifications of idea provided by Asfour et. al. GraspRRT on Robotics and Automation Magazine, 2012
    vector<ContactPoint> Grasper::closeHand(double step, kinematics::BodyNode* target) {
        vector<ContactPoint> resulting_contacts(100);
        if (target == NULL) {
            ECHO("ERROR: Must select object to grasp first!");
            return resulting_contacts;
        }
        int fingers = world->getRobot(robot)->getNode(EEName.c_str())->getNumChildJoints();
        list<kinematics::Joint*> joints;
        vector<int> jointDirections;
        //first build list of joints
        for (int i = 0; i < fingers; i++) {
            //populate list of end-effector joints
            kinematics::Joint* fingerJoint = world->getRobot(robot)->getNode(EEName.c_str())->getChildJoint(i);
            joints.push_back(fingerJoint);
            joints.push_back(fingerJoint->getChildNode()->getChildJoint(0));
            joints.push_back(fingerJoint->getChildNode()->getChildJoint(0)->getChildNode()->getChildJoint(0));

            //populate list of joint directions; finger joint grows - ,while distal and medial grow +
            jointDirections.push_back(-1);
            jointDirections.push_back(1);
            jointDirections.push_back(1);

            //populate end-effector's DoF vector
            hand_dofs.push_back(fingerJoint->getDof(0)->getSkelIndex());
            hand_dofs.push_back(fingerJoint->getChildNode()->getChildJoint(0)->getDof(0)->getSkelIndex());
            hand_dofs.push_back(fingerJoint->getChildNode()->getChildJoint(0)->getChildNode()->getChildJoint(0)->getDof(0)->getSkelIndex());
        }
        vector<bool> colliding_link(joints.size(), false);
        bool grasped = false;
        int jointID = 0;
        int iteration = 0;
        while (!grasped) {
            iteration++;
            grasped = true;
            //iterate through each end-effector joint i.e. 15 joints = 5 x 3 joint per finger
            int link = 0;
            for (list<kinematics::Joint*>::iterator it = joints.begin(); it != joints.end(); it++) {
                //check for collision status
                if (!colliding_link[link]) {
                    grasped = false;
                    kinematics::Joint *j(*it);
                    if (moveLinkWithCollisionChecking(step, jointDirections[link], j, target, resulting_contacts))
                        colliding_link[link] = true;
                }
                link++;
            }
            // perform additional check to allow for better grasping
            if (grasped && iteration > 2) {
                iteration = 0;
                grasped = false;

                for (int i = 0; i < joints.size(); i++) {
                    colliding_link[i] = false;
                }
            }
        }
        return resulting_contacts;
    }
    
    /// Open robot's hand by setting all fingers joints values to 0
    void Grasper::openHand(){
        int fingers = world->getRobot(robot)->getNode(EEName.c_str())->getNumChildJoints();
        int jointID = 0;
        for(int i=0; i < fingers; i++){
            //set all fingers'joint values to 0
            kinematics::Joint *fingerJoint = world->getRobot(robot)->getNode(EEName.c_str())->getChildJoint(i);
            fingerJoint->getDof(0)->setValue(0);
            fingerJoint->getChildNode()->getChildJoint(jointID)->getDof(0)->setValue(0);
            fingerJoint->getChildNode()->getChildJoint(jointID)->getChildNode()->getChildJoint(0)->getDof(0)->setValue(0);
            world->getRobot(robot)->update();
        }
    }
    
    /// Increase a joint value with collision checking
    bool Grasper::moveLinkWithCollisionChecking(double step, int direction, kinematics::Joint* joint, kinematics::BodyNode* target, vector<ContactPoint> contacts){
        bool ret = true;
        
        double oldJointValue = joint->getDof(0)->getValue();
        double newJointValue = oldJointValue + step*direction;
        
        if(newJointValue <= joint->getDof(0)->getMax() && newJointValue >= joint->getDof(0)->getMin()){
            joint->getDof(0)->setValue(newJointValue);
            world->getRobot(robot)->update();
            
            CollisionSkeletonNode* other = world->mCollisionHandle->getCollisionChecker()->getCollisionSkeletonNode(target);
            //check collision against child BodyNode and not current Joint
            if(!world->mCollisionHandle->getCollisionChecker()->getCollisionSkeletonNode(joint->getChildNode())->checkCollision(other, &contacts, contacts.size())){
                ret = false;
            }
            else{
                joint->getDof(0)->setValue(oldJointValue);
                 world->getRobot(robot)->update();
            }
        }
        return ret;
    }
}
