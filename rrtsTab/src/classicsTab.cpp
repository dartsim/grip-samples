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

/* ********************************************************************************************* */
bool classicsTab::rgdNewConfig(Eigen::VectorXd& sample, Eigen::VectorXd near, Event event) {
}

/* ********************************************************************************************* */
pair<double, double> classicsTab::taskError(Eigen::VectorXd& sample, Eigen::VectorXd near, Event event) {

}

/* ********************************************************************************************* */
void classicsTab::plan(const Event& event) {
	
	// Call the path planner	
	PathPlanner <RRT> planner (*mWorld, false, false, 1e-1, 1e6, 0.5);
	vector <int> links;
	for(size_t i = 0; i < 7; i++) links.push_back(i + 6); 
	list <VectorXd> path;
	bool success = planner.planPath(0, links, start, goal, path);
	printf("\n\ndone: %d, traj. length: %lu\n", success, path.size());
	cout << "start: " << start.transpose() << endl;
	cout << "goal: " << goal.transpose() << endl;

	if(success) {
		list <VectorXd>::iterator it = path.begin();
		for(size_t i = 0; it != path.end(); it++, i++) 
			cout << i << ": " << it->transpose() << endl;
	}
}

/* ********************************************************************************************* */
classicsTab::classicsTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, 
		const wxSize& size, long style) : GRIPTab(parent, id, pos, size, style) {

	// Set the sizer
  sizerFull = new wxBoxSizer(wxHORIZONTAL);
  
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
	paramBoxSizer->Add(createTextBox(stepSizeCtrl, "0.01",  "Step size:   "), 0, wxALL, 2);
	paramBoxSizer->Add(createTextBox(goalBiasCtrl, "0.3",   "Goal bias:  "), 0, wxALL, 2);
	paramBoxSizer->Add(createTextBox(numItersCtrl, "100000","Iterations: "), 0, wxALL, 2);
	paramBoxSizer->Add(createTextBox(numNeighborsCtrl, "10","Neighbors:"), 0, wxALL, 2);

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
wxSizer* classicsTab::createTextBox(wxTextCtrl* ctrl, const string& value, const string& def) {

	// Create the sizer and the ctrl.
	wxSizer* box = new wxBoxSizer(wxHORIZONTAL);
	ctrl = new wxTextCtrl(this, ctrl_goalBias, wxString(value.c_str(), wxConvUTF8));

	// Create the static text and add the text and the ctrl to the sizer
	box->Add(new wxStaticText(this, wxID_ANY, wxString(def.c_str(), wxConvUTF8)), 0, wxALL, 6);
	box->Add(ctrl, 0, wxALL, 2);
	return box;
}

/* ********************************************************************************************* */
void classicsTab::OnButton(wxCommandEvent& evt) {
 
	// Get the button and switch on the set symbols
  Event button_num = (Event) evt.GetId();
  switch (button_num) {
	  case button_setStart:
		case button_setGoal:
			setState(button_num);
		break;
	  case button_showStart:
	  case button_showGoal:
			showState(button_num);
		break;
		case button_prepareVideo:
			prepareVideo();
		break;
		case button_algoBaseline:
			plan(button_algoBaseline);
		break;
	} 
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
// Handler for events
BEGIN_EVENT_TABLE(classicsTab, wxPanel)
	EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, classicsTab::OnButton)
	EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, classicsTab::OnSlider)
END_EVENT_TABLE()

// Class constructor for the tab: Each tab will be a subclass of GRIPTab
IMPLEMENT_DYNAMIC_CLASS(classicsTab, GRIPTab)
