/**
 * @file classicsTab.h
 * @author Can Erdogan
 * @date Jan 23, 2013
 * @brief Simple examples of task constrained motion planning with randomized gradient descent
 */

#pragma once

#include <Tabs/GRIPTab.h>
#include <Tabs/GRIPThread.h>
#include <Tools/Constants.h>
#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <Tabs/AllTabs.h>
#include <GRIPApp.h>

#include <collision/CollisionShapes.h>
#include <collision/CollisionSkeleton.h>
#include <dynamics/SkeletonDynamics.h>
#include <dynamics/ContactDynamics.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
#include <planning/PathPlanner.h>
#include <robotics/Object.h>
#include <robotics/Robot.h>

#include <iostream>
#include <fstream>
#include <Eigen/Dense>

/// Tab for examples of classical RRT implementations 
class classicsTab : public GRIPTab {
public: 
	// GUI variables

	/// The callback events
	enum Event {
		button_setStart,
		button_showStart,
		button_setGoal,
		button_showGoal,
		button_prepareVideo,									///< Should be clicked before making a video
		button_algoBaseline,									///< For the baseline algorithm
		button_algoGoalBiased,								///< For the goal-biased algorithm
		button_algoConnect,										///< For the connect algorithm
		button_showTraj,											///< Show the planned trajectory
		button_showRRT,												///< Show the RRT in joint space (only if joints < 3)
		ctrl_stepSize,												///< The step size between nearest neighbor towards random
		ctrl_goalBias,												///< The bias [0,1] to extend towards random node vs. goal
		ctrl_numIters,												///< The maximum number of random samples 
		ctrl_numNeighbors,										///< The # neighbors in bidirectional search to extend
		check_bidir,													///< For bidirectional implementation 
		check_short 													///< To shorten goal-biased/connect paths
	};

  wxSizer* sizerFull;											///< The sizer in charge of the entire frame
	wxTextCtrl* stepSizeCtrl;								///< To set the step size in baseline/gb/connect
	wxTextCtrl* goalBiasCtrl;								///< To set the goal biase in gb/connect
	wxTextCtrl* numItersCtrl;								///< To set the # max iterations in baseline/gb/connect
	wxTextCtrl* numNeighborsCtrl;						///< To set the # nearest neighbors (bidirectional) 
	std::vector <Eigen::VectorXd> baked;		///< The saved states after a path is executed for drawing

public:
	// Planning variables

	Eigen::VectorXd start;									///< The start state
	Eigen::VectorXd goal;										///< The goal state
	double stepSize;												///< Step size between nearest neighbor towards random
	double goalBise;												///< The bias [0,1] to extend towards random node vs. goal
	size_t numIters;												///< The maximum number of random samples 
	size_t numNeighbors;                    ///< The # neighbors in bidirectional search to extend

public:
	// Mandatory interface functions

  classicsTab(){};									///< Default constructor
  classicsTab(wxWindow * parent, wxWindowID id = -1, const wxPoint & pos = wxDefaultPosition,
		const wxSize & size = wxDefaultSize, long style = wxTAB_TRAVERSAL);		
  virtual ~classicsTab(){};				///< Destructor
  void OnButton(wxCommandEvent &evt);			///< Handle button events
  void OnSlider(wxCommandEvent &evt) {}		///< Necessary for compilation (bug!)

	/// Creates a sizer that includes a text ctrl and a static text explaining expected input
	wxSizer* createTextBox(wxTextCtrl* ctrl, const std::string& value, const std::string& definition);

public:
	// Tab functions

	void setState (const Event& event);			///< Set start/goal state in local field and file
	void showState (const Event& event);		///< Show start/goal state from local field or file
	void drawPath (const list <Eigen::VectorXd>& path);													///< Execute path
	void prepareVideo ();										///< Prepare the data structures of frame for a video

public:
	// Project functions

	void plan (const Event& event);					///< Perform rgd with the given constraint
	bool rgdNewConfig(Eigen::VectorXd& sample, Eigen::VectorXd near, Event ev); ///< RGD algorithm
	std::pair<double,double> 
	taskError(Eigen::VectorXd& sample, Eigen::VectorXd near, Event ev);	///< Task space error

public:
	// wxWidget stuff

  DECLARE_DYNAMIC_CLASS(classicsTab)
	DECLARE_EVENT_TABLE()
};

