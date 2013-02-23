/**
 * @file classicsTab.cpp
 * @author Can Erdogan
 * @date Jan 23, 2013
 * @brief Simple examples of task constrained motion planning with randomized gradient descent
 */

#include "classicsTab.h"

using namespace Eigen;
using namespace planning;
using namespace std;

#define sBool(x) ((x) ? ("yes") : ("no"))

/* ********************************************************************************************* */
void classicsTab::plan(const Event& event) {
	
	// ======================================================================
	// Setup for the plan call

	// Sanity check that a world exists and there is a robot
	if((mWorld == NULL) || (mWorld->getNumRobots() == 0)) {
		frame->GetStatusBar()->SetStatusText(wxT("ERROR: There is no world or a robot to plan with!"), 1);
		timer.Start(1000, 1);	
		return;
	}

	// Set the call options according to the specific plan button
	bool connect = false;
	string plannerName;
	double executedGoalBias = goalBias; 
	if(event == button_algoBaseline) { 
		plannerName = string("Baseline");
		executedGoalBias = 0.0;
	}
	else if(event == button_algoConnect)  {
		connect = true;
		plannerName = string("Connect");
	}
	else if(event == button_algoGoalBiased) {
		plannerName = string("Biased");
	}

	// ======================================================================
	// Make the call and visualize the result

	// Print a status message with the input arguments:
	char message [128];
	sprintf(message, "%s: %g steps, %g nodes, %g bias, %s con., %s bi., %s short", plannerName.c_str(),
		stepSize, (double) numNodes, executedGoalBias, sBool(connect), sBool(bidirectional), sBool(shortenTraj));
	frame->GetStatusBar()->SetStatusText(wxString(message, wxConvUTF8), 0);

	// Start the timer
	struct timeval startTime, endTime;
	gettimeofday(&startTime, NULL);

	// Call the path planner	
	path.clear();
	printf("Calling the planner with the arguments: stepSize: %lf, numNodes: %lu, goalBias: %lf\n", stepSize, 
			numNodes, executedGoalBias);
	planner = PathPlanner <RRT> (*mWorld, false, connect, stepSize, numNodes, executedGoalBias);
	vector <int> links;
	for(size_t i = 0; i < 7; i++) links.push_back(i + 6); 
	bool success = planner.planPath(0, links, start, goal, path);
	printf("\n\ndone: %d, traj. length: %lu\n", success, path.size());
	cout << "start: " << start.transpose() << endl;
	cout << "goal: " << goal.transpose() << endl;

	// End the timer
  gettimeofday(&endTime, NULL);
  long seconds  = endTime.tv_sec  - startTime.tv_sec;
  long useconds = endTime.tv_usec - startTime.tv_usec;
	long mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;

	// Shorten the path if necessary
	if(shortenTraj) {
		PathShortener pathShortener (mWorld, 0, links, stepSize);
		pathShortener.shortenPath(path);
	}

	// Draw the path
	if(success) {

		// Update the message
		sprintf(message, "%s: path found with %lu nodes in %g ms. RRT(s) size: %1.2e.", plannerName.c_str(),
			path.size(), (double) mtime, (double) planner.start_rrt->getSize() + 
			+ (planner.goal_rrt == NULL ? 0 : planner.goal_rrt->getSize()));
		frame->GetStatusBar()->SetStatusText(wxString(message, wxConvUTF8), 0);
		timer.Start(2000, 0);	

		// Draw the execution of the path
		drawPath(path);
	}
	else {
		sprintf(message, "%s: could not find path.", plannerName.c_str());
		frame->GetStatusBar()->SetStatusText(wxString(message, wxConvUTF8), 0);
		timer.Start(1000, 0);	
	}
}

/* ********************************************************************************************* */
classicsTab::classicsTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, 
		const wxSize& size, long style) : GRIPTab(parent, id, pos, size, style) {

	// Set the sizer
  sizerFull = new wxBoxSizer(wxHORIZONTAL);
  
 	// =====================================================================
	// 0. Setup the initial values

	// Choose the values
	stepSize = 1e-2;
	goalBias = 0.3;
	numNodes = 1e6;
	bidirectional = true;
	shortenTraj = true;

	// Write them in text mode for the controllers
	char stepSizeStr [16], goalBiasStr [16], numNodesStr [16];
	sprintf(stepSizeStr, "%1.2lf", stepSize);
	sprintf(goalBiasStr, "%1.2lf", goalBias);
	sprintf(numNodesStr, "%lu", numNodes);

 	// =====================================================================
	// 1. Create the algorithm box for baseline, goal-biased and connect

	wxStaticBox* algoBox = new wxStaticBox(this, -1, wxT("Algorithms"));
	wxStaticBoxSizer* algoBoxSizer = new wxStaticBoxSizer(algoBox, wxVERTICAL);
	algoBoxSizer->Add(new wxButton(this, button_algoBaseline, wxT("Baseline")), 0, wxALL, 4);
	algoBoxSizer->Add(new wxButton(this, button_algoGoalBiased, wxT("Goal-Biased")), 0, wxALL, 4);
	algoBoxSizer->Add(new wxButton(this, button_algoConnect, wxT("Connect")), 0, wxALL, 4);

	// Make the sizer a bit wider
	wxSize currSize = algoBoxSizer->GetMinSize();
	algoBoxSizer->SetMinSize(currSize.x + 10, currSize.y);

 	// =====================================================================
	// 2. Create the options box for bidirectional and shortening

	wxStaticBox* optsBox = new wxStaticBox(this, -1, wxT("Options"));
	wxStaticBoxSizer* optsBoxSizer = new wxStaticBoxSizer(optsBox, wxVERTICAL);
	wxCheckBox* temp = new wxCheckBox(this, check_bidir, wxT("Bidirectional"));
	temp->SetValue(true);
	optsBoxSizer->Add(temp, 0, wxALL, 4);
	temp = new wxCheckBox(this, check_short, wxT("Shortening"));
	temp->SetValue(true);
	optsBoxSizer->Add(temp, 0, wxALL, 4);
	
 	// =====================================================================
	// 3. Create the setup box for start and goal configurations

	wxStaticBox* setupBox = new wxStaticBox(this, -1, wxT("Setup"));
	wxStaticBoxSizer* setupBoxSizer = new wxStaticBoxSizer(setupBox, wxHORIZONTAL);
	wxSizer* setupBoxSizerCol1 = new wxBoxSizer(wxVERTICAL);
	wxSizer* setupBoxSizerCol2 = new wxBoxSizer(wxVERTICAL);
	setupBoxSizerCol1->Add(new wxButton(this, button_setStart, wxT("Set Start")), 0, wxALL, 4);
	setupBoxSizerCol1->Add(new wxButton(this, button_showStart, wxT("Show Start")), 0, wxALL, 4);
	setupBoxSizerCol2->Add(new wxButton(this, button_setGoal, wxT("Set Goal")), 0, wxALL, 4);
	setupBoxSizerCol2->Add(new wxButton(this, button_showGoal, wxT("Show Goal")), 0, wxALL, 4);
	setupBoxSizerCol2->Add(new wxButton(this, button_prepareVideo, wxT("Video")), 0, wxALL, 4);
	setupBoxSizer->Add(setupBoxSizerCol1, 1, wxALL | wxEXPAND, 2);
	setupBoxSizer->Add(setupBoxSizerCol2, 1, wxALL | wxEXPAND, 2);

 	// =====================================================================
	// 4. Create the parameters box for step size, goal bias and etc.

	wxStaticBox* paramBox = new wxStaticBox(this, -1, wxT("Parameters"));
	wxStaticBoxSizer* paramBoxSizer = new wxStaticBoxSizer(paramBox, wxVERTICAL);
	paramBoxSizer->Add(createTextBox(stepSizeCtrl, stepSizeStr,  "Step size:   "), 0, wxALL, 2);
	paramBoxSizer->Add(createTextBox(goalBiasCtrl, goalBiasStr,   "Goal bias:  "), 0, wxALL, 2);
	paramBoxSizer->Add(createTextBox(numItersCtrl, numNodesStr ,"Iterations: "), 0, wxALL, 2);

 	// =====================================================================
	// 5. Create the display box to view trajectories and joint space RRTs

	wxStaticBox* dispBox = new wxStaticBox(this, -1, wxT("Display"));
	wxStaticBoxSizer* dispBoxSizer = new wxStaticBoxSizer(dispBox, wxVERTICAL);
	dispBoxSizer->Add(new wxButton(this, button_showTraj, wxT("Show Trajectory")), 0, wxALL, 4);
	dispBoxSizer->Add(new wxButton(this, button_showRRT, wxT("Show RRT!")), 0, wxALL, 4);
	
 	// =====================================================================
	// 6. Create empty far right container to look nice

	wxStaticBox* emptyBox = new wxStaticBox(this, -1, wxT(""));
	wxStaticBoxSizer* emptyBoxSizer = new wxStaticBoxSizer(emptyBox, wxVERTICAL);

 	// =====================================================================
	// 7. Add all sizers to the full sizer

	// Create the sizer that controls the tab panel
	wxBoxSizer* sizerFull = new wxBoxSizer (wxHORIZONTAL);
	
	// Add the sides
	sizerFull->Add(paramBoxSizer, 0, wxALL | wxEXPAND, 5);
	sizerFull->Add(optsBoxSizer, 0, wxALL | wxEXPAND, 5);
	sizerFull->Add(setupBoxSizer, 0, wxALL | wxEXPAND, 5);
	sizerFull->Add(algoBoxSizer, 0, wxALL | wxEXPAND, 5);
	sizerFull->Add(dispBoxSizer, 0, wxALL | wxEXPAND, 5);
	sizerFull->Add(emptyBoxSizer, 2, wxALL | wxEXPAND, 5);

	// Set the camera zoom
	viewer->camRadius = 5.0;
	viewer->UpdateCamera();

	// Set the full sizer as the sizer of this tab
	SetSizer(sizerFull);
}

/* ********************************************************************************************* */
wxSizer* classicsTab::createTextBox(wxTextCtrl*& ctrl, const string& value, const string& def) {

	// Create the sizer and the ctrl.
	wxSizer* box = new wxBoxSizer(wxHORIZONTAL);
	ctrl = new wxTextCtrl(this, ctrl_goalBias, wxString(value.c_str(), wxConvUTF8), wxDefaultPosition, 
		wxDefaultSize, wxTE_PROCESS_ENTER);

	// Create the static text and add the text and the ctrl to the sizer
	box->Add(new wxStaticText(this, wxID_ANY, wxString(def.c_str(), wxConvUTF8)), 0, wxALL, 6);
	box->Add(ctrl, 0, wxALL, 2);
	return box;
}

/* ********************************************************************************************* */
void classicsTab::OnCheckBox(wxCommandEvent& evt) {

	size_t evtId = evt.GetId();
	if(evtId == check_bidir)
		bidirectional = (bool) evt.GetSelection();
	else if(evtId == check_short)
		shortenTraj = (bool) evt.GetSelection();
}

/* ********************************************************************************************* */
void classicsTab::OnButton(wxCommandEvent& evt) {
 
	// Get the button and switch on the set symbols
  Event button_num = (Event) evt.GetId();
  switch (button_num) {

		// Setup plan arguments
	  case button_setStart:
		case button_setGoal:
			setState(button_num);
		break;
	  case button_showStart:
	  case button_showGoal:
			showState(button_num);
		break;

		// Carry out a specific plan
		case button_algoBaseline: 
		case button_algoGoalBiased:
		case button_algoConnect: 
			plan(button_num); break;

		// Showing results after plan is completed
		case button_prepareVideo: prepareVideo(); break;
		case button_showTraj: drawPath(path); break;
		case button_showRRT:
			if(path.empty()) {
				frame->GetStatusBar()->SetStatusText(wxT("ERROR: There is no planned path!"), 1);
				timer.Start(1000, 1);	
			}
			else if(path.front().size() != 2) {
				frame->GetStatusBar()->SetStatusText(wxT("ERROR: The # of DOFs should be 2 or 3!"), 1);
				timer.Start(1000, 1);	
			}
			else planner.start_rrt->draw();
		break;
	} 
}

/* ********************************************************************************************* */
void classicsTab::BarTimer::Start(size_t ms, size_t field_) {
	field = field_;
	wxTimer::Start(ms);
}
	
/* ********************************************************************************************* */
void classicsTab::BarTimer::Notify() {
	frame->GetStatusBar()->SetStatusText(wxT(""), field);
	Stop();
}

/* ********************************************************************************************* */
void classicsTab::setState(const Event& event) {

	// Get the current configuration and save to the local field
	assert(((mWorld != NULL) && (mWorld->getNumRobots() > 0)) && "Bad world");
	(event == button_setStart ? start : goal) = mWorld->getRobot(0)->getQuickDofs();
	
	// Save it to the file
	ofstream file (event == button_setGoal ? ".goal" : ".start");
	file << mWorld->getRobot(0)->getQuickDofs().transpose(); 
	file.close();
}

/* ********************************************************************************************* */
void classicsTab::showState(const Event& event) {

	// Read from the file
	ifstream file (event == button_showGoal ? ".goal" : ".start", ifstream::in);
	assert(file.is_open() && "file not open");
	assert(((mWorld != NULL) && (mWorld->getNumRobots() > 0)) && "Bad world");
	size_t numDofs = mWorld->getRobot(0)->getNumQuickDofs();
	Eigen::VectorXd temp (numDofs);
	for(size_t i = 0; i < numDofs; i++) 
		file >> temp(i); 
	file.close();

	// Update the local field and screen
	(event == button_showStart ? start : goal) = temp;
	mWorld->getRobot(0)->setQuickDofs(temp);
	mWorld->getRobot(0)->update();
	viewer->DrawGLScene(); 
}

/* ********************************************************************************************* */
void classicsTab::drawPath (const list <Eigen::VectorXd>& path) {

	const size_t fps = 30;
	for(list<Eigen::VectorXd>::const_iterator it = path.begin(); it != path.end(); it++) {

		// Update the screen
		mWorld->getRobot(0)->setQuickDofs(*it);
		mWorld->getRobot(0)->update();
		viewer->DrawGLScene(); 
		usleep(1e6 / fps);
		
		// Save the world
		baked.push_back(mWorld->getState());
	}
}

/* ********************************************************************************************* */
void classicsTab::prepareVideo () {

	// Reset the data structure for the videos
  frame->InitTimer(string("Planner"), mWorld->mTimeStep);

  // Set the saved world states in that data structure (in frame, called timeVector)
  int numsteps = baked.size();
  for( int i = 0; i < numsteps; ++i ) {
		mWorld->setState(baked[i]);
    for (int j = 0; j < mWorld->getNumRobots(); j++) {
      mWorld->getRobot(j)->update();
			mWorld->getSkeleton(j)->setImmobileState(true);
		}
    for (int j = 0; j < mWorld->getNumObjects(); j++) 
      mWorld->getObject(j)->update();
    frame->AddWorld(mWorld);
  }
}

/* ********************************************************************************************* */
void classicsTab::OnText(wxCommandEvent &evt) {

	// Set the corresponding value to the text controller and reset its 'modified' state
	if(stepSizeCtrl->IsModified()) {
		stepSizeCtrl->SetModified(false);
		stepSize = atof(string(stepSizeCtrl->GetValue().mb_str()).c_str());
	}
	else if(goalBiasCtrl->IsModified()) {
		goalBiasCtrl->SetModified(false);
		goalBias = atof(string(goalBiasCtrl->GetValue().mb_str()).c_str());
	}
	else if(numItersCtrl->IsModified()) {
		numItersCtrl->SetModified(false);
		numNodes = atoi(string(numItersCtrl->GetValue().mb_str()).c_str());
	}
}

/* ********************************************************************************************* */
// Handler for events
BEGIN_EVENT_TABLE(classicsTab, wxPanel)
	EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, classicsTab::OnButton)
	EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_CHECKBOX_CLICKED, classicsTab::OnCheckBox)
	EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, classicsTab::OnSlider)
	EVT_TEXT_ENTER (wxID_ANY, classicsTab::OnText)
END_EVENT_TABLE()

// Class constructor for the tab: Each tab will be a subclass of GRIPTab
IMPLEMENT_DYNAMIC_CLASS(classicsTab, GRIPTab)
