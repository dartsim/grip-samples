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
/**
 * @file JointMover.cpp
 * @brief Jacobian-based planning handling both translations and rotations
 * @author Juan C. Garcia, Justin Smith in collaboration with Arash Rouhani made modifications 
 *         to original code provided by Ana C. Huaman Quispe 
 */
#include <iostream>
#include <stdlib.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/Transformation.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>
#include <vector>
#include <Eigen/LU>
#include <list>
#include <dart/dynamics/ContactDynamics.h>
#include <dart/collision/CollisionSkeleton.h>
#include <dart/dynamics/SkeletonDynamics.h>
#include "JointMover.h"

using namespace std;
using namespace Eigen;

const double dt = 0.1;
JointMover::JointMover( simulation::World &_world, dynamics::SkeletonDynamics* robot, const std::vector<int> &_links,  std::string _EEName,
		       double _configStep)
  : mConfigStep(_configStep), mWorld(_world), mRobot(robot) {

  mLinks = _links;
  mMaxIter = 100;
  mWorkspaceThresh = 0.02;
  mEENode = (dynamics::BodyNodeDynamics*)mRobot->getNode(_EEName.c_str());
}

// Method returns either a translational Jacobian, or a full trans+rot Jacobian
MatrixXd JointMover::GetPseudoInvJac(bool both) {
  MatrixXd Jaclin = mEENode->getJacobianLinear().topRightCorner( 3, mLinks.size());
  MatrixXd Jac;
  // handle both translations and rotations
  if(both) {
      MatrixXd Jacang = mEENode->getJacobianAngular().topRightCorner(3, mLinks.size());
      Jac.resize(Jaclin.rows() + Jacang.rows(), Jaclin.cols());
      Jac << Jaclin, Jacang;
  }// otherwise, just handle translations
  else {
      Jac = Jaclin;
  }
  MatrixXd JacT = Jac.transpose();
  MatrixXd JJt = (Jac*JacT);
  FullPivLU<MatrixXd> lu(JJt);
  
  return JacT*(lu.inverse());
}

// Method performs a step towards target; such target could  
// be a 3D vector (X,Y,Z) or a 6D vector (X,Y,Z,R,P,Y)
VectorXd JointMover::OneStepTowardsXYZRPY( VectorXd _q, VectorXd _targetXYZRPY) {
  assert(_targetXYZRPY.size() >= 3);
  assert(_q.size() > 3);
  bool both = (_targetXYZRPY.size() == 6);
  //GetXYZ also updates the config to _q, so Jaclin use an updated value
  VectorXd delta = _targetXYZRPY - GetXYZRPY(_q, both); 
  //if target also specifies roll, pitch and yaw compute entire Jacobian; otherwise, just translational Jacobian
  VectorXd dConfig = GetPseudoInvJac(both)*delta;
  
  // Constant to not let vector to be larger than mConfigStep
  double alpha = min((mConfigStep/dConfig.norm()), 1.0);
  dConfig = alpha*dConfig;
  return _q + dConfig;
}

// Method performs a Jacobian towards specified target; such target could  
// be a 3D vector (X,Y,Z) or a 6D vector (X,Y,Z,R,P,Y)
bool JointMover::GoToXYZRPY( VectorXd _qStart, VectorXd _targetXYZRPY, VectorXd &_qResult, std::list<Eigen::VectorXd> &path) {
  _qResult = _qStart;
  bool both = (_targetXYZRPY.size() == 6);
  // GetXYZ also updates the config to _qResult, so Jaclin use an updated value
  VectorXd delta = _targetXYZRPY - GetXYZRPY(_qResult, both); 
  
  int iter = 0;
  while( delta.norm() > mWorkspaceThresh && iter < mMaxIter ) {
    _qResult = OneStepTowardsXYZRPY(_qResult, _targetXYZRPY);
    path.push_back(_qResult);
    delta = (_targetXYZRPY - GetXYZRPY(_qResult, both) );
    //PRINT(delta.norm());
    iter++;
  }
  return iter < mMaxIter;
}

// Method to compute location of given a vector of joint configurations; note the
// returned vector could be 3D (X,Y,Z) or 6D(X,Y,Z,R,P,Y)
VectorXd JointMover::GetXYZRPY( VectorXd _q, bool both ) {
  // Get current XYZ position
  mRobot->setConfig(mLinks, _q);
  
  MatrixXd qTransform = mEENode->getWorldTransform();
  VectorXd qXYZRPY;
  
  //return a 6D vector if both x,y,z and r,p,y must be computed
  if(both){
      Matrix3d rotM = qTransform.topLeftCorner(3,3);
      VectorXd rot = rotM.eulerAngles(0,1,2);
      qXYZRPY.resize(6); 
      qXYZRPY << qTransform(0,3), qTransform(1,3), qTransform(2,3), rot(0), rot(1), rot(2);
  }
  else{
      qXYZRPY.resize(3); 
      qXYZRPY << qTransform(0,3), qTransform(1,3), qTransform(2,3);
  }
  return qXYZRPY;
}


// Method to compute distance between configs in joint space
double JointMover::jointSpaceDistance(VectorXd _q1, VectorXd _q2) {
  // This is the infinite norm
  return (_q2-_q1).cwiseAbs().maxCoeff();
}

// Method to achieve simple joints movements
VectorXd JointMover::jointSpaceMovement(VectorXd _qStart, VectorXd _qGoal) {
  VectorXd diff = (_qGoal-_qStart);
  for(int i = 0; i < diff.size(); i++){
    diff[i] = max(-jointSpeeds*dt, min(jointSpeeds*dt, diff[i]));
  }
  return _qStart + diff;
}
