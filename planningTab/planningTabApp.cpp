/**
 * @file pushDemoTabApp.h
 * @brief Creates application for pushDemoTab
 * @author A. Huaman Q.
 */
#include "GRIPApp.h"
#include "planningTab.h"

extern wxNotebook* tabView;

class planningTabApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new planningTab(tabView), wxT("Planning Tab"));
	}
};

IMPLEMENT_APP(planningTabApp)
