/**
 * @file planningTabApp.h
 * @brief Creates application for planningTab
 * @author A. Huaman Q.
 */
#include "GRIPApp.h"
#include "planningTab.h"

extern wxNotebook* tabView;

class planningTabApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new planningTab(tabView), wxT("PlanningTab"));
	}
};

IMPLEMENT_APP(planningTabApp)
