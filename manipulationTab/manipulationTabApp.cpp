/**
 * @file pushDemoTabApp.h
 * @brief Creates application for pushDemoTab
 * @author A. Huaman Q.
 */
#include "GRIPApp.h"
#include "manipulationTab.h"

extern wxNotebook* tabView;

class manipulationTabApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new manipulationTab(tabView), wxT("Manipulation"));
	}
};
IMPLEMENT_APP(manipulationTabApp)
