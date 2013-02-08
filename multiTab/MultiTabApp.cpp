#include "GRIPApp.h"
#include "../visualizationTab/VisualizationTab.h"
#include "../planningTab/planningTab.h"

extern wxNotebook* tabView;

class MultiTabApp : public GRIPApp {
    virtual void AddTabs() {
        tabView->AddPage(new planningTab(tabView), wxT("Planning"));
        tabView->AddPage(new VisualizationTab
        (tabView), wxT("Visualization"));
    }
};

IMPLEMENT_APP(MultiTabApp)
