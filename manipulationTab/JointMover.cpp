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
 * @brief Jacobian-based planning handling only translations
 * @author Juan C. Garcia in collaboration with Arash Rouhani made minor modifications 
 * to code provided by Ana C. Huaman Quispe 
 */
#include <iostream>
#include <stdlib.h>
#include <robotics/Robot.h>
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
JointMover::JointMover( robotics::World &_world, robotics::Robot* robot, const std::vector<int> &_links,  std::string _EEName,
		       double _configStep)
  : mConfigStep(_configStep), mWorld(_world), mRobot(robot) {

  mLinks = _links;
  mMaxIter = 100;
  mWorkspaceThresh = _configStep;
  mEENode = (dynamics::BodyNodeDynamics*)mRobot->getNode(_EEName.c_str());
}

MatrixXd JointMover::GetPseudoInvJac() {
   // Precalculate pseduojacobian
  MatrixXd Jaclin = mEENode->getJacobianLinear().topRightCorner( 3, mLinks.size());
  MatrixXd JaclinT = Jaclin.transpose();
  MatrixXd JJt = (Jaclin*JaclinT);
  FullPivLU<MatrixXd> lu(JJt);
  
  return JaclinT*( lu.inverse() );;
}

VectorXd JointMover::OneStepTowardsXYZ( VectorXd _q, VectorXd _targetXYZ) {
  assert(_targetXYZ.size() == 3);
  assert(_q.size() > 3);
  VectorXd dXYZ = _targetXYZ - GetXYZ(_q); // GetXYZ also updates the config to _q, so Jaclin use an updated value
  VectorXd dConfig = GetPseudoInvJac()*dXYZ;

  double alpha = min((mConfigStep/dConfig.norm()), 1.0); // Constant to not let vector to be larger than mConfigStep
  dConfig = alpha*dConfig;
  return _q + dConfig;
}

bool JointMover::GoToXYZ( VectorXd _qStart, VectorXd _targetXYZ, VectorXd &_qResult, std::list<Eigen::VectorXd> &path) {
  _qResult = _qStart;
  mRobot->update();
  VectorXd dXYZ = _targetXYZ - GetXYZ(_qResult); // GetXYZ also updates the config to _qResult, so Jaclin use an updated value
  int iter = 0;
  while( dXYZ.norm() > mWorkspaceThresh && iter < mMaxIter ) {
    _qResult = OneStepTowardsXYZ(_qResult, _targetXYZ);
    path.push_back(_qResult);
    mRobot->update();
    dXYZ = (_targetXYZ - GetXYZ(_qResult) );
    //PRINT(dXYZ.norm());
    iter++;
  }
  
  mRobot->update();
  return iter < mMaxIter;
}

VectorXd JointMover::GetXYZ( VectorXd _q ) {
  // Get current XYZ position
  mRobot->setConfig(mLinks, _q);
  mRobot->update();
  
  MatrixXd qTransform = mEENode->getWorldTransform();
  VectorXd qXYZ(3); qXYZ << qTransform(0,3), qTransform(1,3), qTransform(2,3);
  return qXYZ;
}

double JointMover::jointSpaceDistance(VectorXd _q1, VectorXd _q2) {
  // This is the infinite norm
  return (_q2-_q1).cwiseAbs().maxCoeff();
}

VectorXd JointMover::jointSpaceMovement(VectorXd _qStart, VectorXd _qGoal) {
  VectorXd diff = (_qGoal-_qStart);
  for(int i = 0; i < diff.size(); i++){
    diff[i] = max(-jointSpeeds*dt, min(jointSpeeds*dt, diff[i]));
  }
  return _qStart + diff;
}
