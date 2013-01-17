/**
 * @file pushDemoTabApp.h
 * @brief Creates application for pushDemoTab
 * @author A. Huaman Q.
 */
#include "GRIPApp.h"
#include "pushDemoTab.h"

extern wxNotebook* tabView;

class pushDemoTabApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new pushDemoTab(tabView), wxT("PlanningTab"));
	}
};

IMPLEMENT_APP(pushDemoTabApp)
