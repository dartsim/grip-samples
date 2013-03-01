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
    
    void Grasper::init(std::vector<int>& d, Eigen::VectorXd& start, kinematics::BodyNode* node){
        dofs = d; 
        startConfig = start;
        objectNode = node;
        
        //initialize JointMover with end effector and arm's DoFs
        jm = new JointMover(*world, robot, dofs, EEName, 0.01);
    }
    
    void Grasper::plan(list<VectorXd> &path, vector<int> &totalDofs) {
        //find closest point in target object; grasp target point
        int min = calculateMinDistance(graspPoint); 
        VectorXd goalPose(6);
        
        //perform translation Jacobian towards grasping point computed
        jm->GoToXYZ(startConfig, graspPoint, goalPose, path);
        
        //try to close hand;  //QUICK FIX: expand each vector in path to be 21 DOFs
        totalDofs = dofs;
        closeHand(0.1, objectNode);
        //merge DOFS
        totalDofs.insert(totalDofs.end(), hand_dofs.begin(), hand_dofs.end());
        //increase size of every vector in path
        for(list<VectorXd>::iterator it = path.begin(); it != path.end(); it++){
            VectorXd & v (*it);
            v.conservativeResize(totalDofs.size()); v.segment(dofs.size(),hand_dofs.size()) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
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
 
    double Grasper::calculateMinDistance(Vector3d &closest){
        //1. get collision meshes and vertices
        kinematics::Shape* shape = objectNode->getShape();
        const aiScene* sc = shape->getVizMesh();
        
        if (shape->getCollisionMesh() != NULL) { sc = shape->getCollisionMesh(); }
        double min_distance = -1;
         
        if(sc != NULL){
            const aiNode* nd = sc->mRootNode;
            Matrix4d worldTrans = objectNode->getWorldTransform();
            const struct aiMesh* mesh = sc->mMeshes[nd->mMeshes[0]];
           
            //find closest vertex
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
    
    bool Grasper::closeHand(double step, kinematics::BodyNode* target){
        int fingers = world->getRobot(robot)->getNode(EEName.c_str())->getNumChildJoints();
        int cycles = 0;
        int max_cycles = 8;
        set<int> colliding_fingers;
        bool already = false;
        int jointID = 0;
        //close hand until all finger collide
        while(colliding_fingers.size() < fingers && cycles < max_cycles){
            for(int i=0; i < fingers; i++){
                //if finger hasn't collided, keep adding offset
                if(colliding_fingers.find(i) == colliding_fingers.end()){
                    //for each finger, get joint value of finger joint, finger's medial and distal
                    kinematics::Joint *fingerJoint = world->getRobot(robot)->getNode(EEName.c_str())->getChildJoint(i);
                    
                    double oldFingerJointValue = fingerJoint->getDof(0)->getValue();
                    double oldMedialJointValue = fingerJoint->getChildNode()->getChildJoint(jointID)->getDof(0)->getValue();
                    double oldDistalJointValue = fingerJoint->getChildNode()->getChildJoint(jointID)->getChildNode()->getChildJoint(0)->getDof(0)->getValue();
                    
                    //QUICK FIX: build hand DOFs once
                    if(!already){
                        //build hand DoFs
                        hand_dofs.push_back(fingerJoint->getDof(0)->getSkelIndex());//world->getRobot(robot)->getNode(EEName.c_str())->getDof(0)->getSkelIndex());
                        hand_dofs.push_back(fingerJoint->getChildNode()->getChildJoint(jointID)->getDof(0)->getSkelIndex());//world->getRobot(robot)->getNode(EEName.c_str())->getChildNode(0)->getSkelIndex());
                        hand_dofs.push_back(fingerJoint->getChildNode()->getChildJoint(jointID)->getChildNode()->getChildJoint(0)->getDof(0)->getSkelIndex());//world->getRobot(robot)->getNode(EEName.c_str())->getChildNode(0)->getChildNode(0)->getSkelIndex());
                    }
                    //add increment to close hand; 3x to finger, 2x to medial and reg. to distal
                    if(oldFingerJointValue >= (-70)){fingerJoint->getDof(0)->setValue((double)oldFingerJointValue - 2.5*step);}
                    if(oldMedialJointValue <= 15.0){fingerJoint->getChildNode()->getChildJoint(jointID)->getDof(0)->setValue((double)oldMedialJointValue + step);}
                    if(oldDistalJointValue <= 15.0){fingerJoint->getChildNode()->getChildJoint(jointID)->getChildNode()->getChildJoint(0)->getDof(0)->setValue((double)oldDistalJointValue + step);}
                    world->getRobot(robot)->update();  
                    
                    // check for target object; could be just opening and closing hand
                    if(target != NULL){
                        //check collisions for each finger after each move; need to check Medial and Distal joints/nodes
                        kinematics::BodyNode *medial = fingerJoint->getChildNode()->getChildJoint(jointID)->getChildNode();
                        kinematics::BodyNode *distal = fingerJoint->getChildNode()->getChildJoint(jointID)->getChildNode()->getChildJoint(0)->getChildNode();

                        vector<ContactPoint> contactPoints(100);
                        CollisionSkeletonNode* other = world->mCollisionHandle->getCollisionChecker()->getCollisionSkeletonNode(objectNode);
                        int distalColls = world->mCollisionHandle->getCollisionChecker()->getCollisionSkeletonNode(distal)->checkCollision(other, &contactPoints, contactPoints.size());
                        int medialColls = world->mCollisionHandle->getCollisionChecker()->getCollisionSkeletonNode(medial)->checkCollision(other, &contactPoints, contactPoints.size());
                        //if fingerColls > 0, this finger has touched object i.e add to set; give some time to other fingers to collide too
                        if(distalColls && medialColls && cycles > max_cycles/2){ colliding_fingers.insert(i);};
                    }
                }
            }
            already = true;
            cycles++;
        }
        return (colliding_fingers.size() == fingers);
    }
    
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
}
