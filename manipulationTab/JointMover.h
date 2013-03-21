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
#ifndef _JointMover_H_
#define _JointMover_H_

#include <iostream>
#include <Eigen/Core>
#include <vector>
#include <dynamics/BodyNodeDynamics.h>
#include <dynamics/SkeletonDynamics.h>
#include <robotics/World.h>
// Macros
#define PRINT(x) std::cout << #x << " = " << x << std::endl;
#define ECHO(x) std::cout << x << std::endl;

using namespace std;
using namespace Eigen;
// The speed of each joint, note that the joint values are between -120 to
// 120 on the robot arm
const double jointSpeeds = 5.0; // degrees/second

class JointMover {
  private:
    /// Member variables
    double mConfigStep;
    robotics::World &mWorld;
    robotics::Robot* mRobot;
    std::vector<int> mLinks;
    
    double mWorkspaceThresh;

    dynamics::BodyNodeDynamics* mEENode;
    int mMaxIter;
    
  public:
    JointMover( robotics::World &_world, robotics::Robot* robot, const std::vector<int> &_links,  std::string _EEName,
        double _configStep);
    MatrixXd GetPseudoInvJac(bool both);
    
    bool GoToXYZRPY( VectorXd _qStart, VectorXd _targetXYZ, VectorXd &_qResult, std::list<Eigen::VectorXd> &path );

    VectorXd OneStepTowardsXYZRPY( VectorXd _q, VectorXd _targetXYZ);

    VectorXd GetXYZRPY( VectorXd _q , bool both);
    
    double jointSpaceDistance(VectorXd _q1, VectorXd _q2);
    
    VectorXd jointSpaceMovement(VectorXd _qStart, VectorXd _qGoal);
};

#endif

