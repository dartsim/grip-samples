/**
 * @file VisualizationTabApp.h
 * @brief Creates application for VisualizationTab
 * @author Saul Reynolds-Haertle
 */
#include "GRIPApp.h"
#include "VisualizationTab.h"

extern wxNotebook* tabView;

class VisualizationTabApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new VisualizationTab(tabView), wxT("Visualization"));
	}
};

IMPLEMENT_APP(VisualizationTabApp)
