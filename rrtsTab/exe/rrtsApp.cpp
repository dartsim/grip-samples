/**
 * @file rrtsApp.h
 * @brief Creates application for the classic and task constrained RRT tabs
 * @author Can Erdogan
 * @date Jan 30, 2013
 */

#include "GRIPApp.h"
#include "classicsTab.h"

extern wxNotebook* tabView;

/**
 * @class cubesTabApp
 * @brief Define Tabs for project
 */
class cubesTabApp : public GRIPApp {
  virtual void AddTabs() {
    tabView->AddPage(new classicsTab(tabView), wxT("Classics"));
  }
};

IMPLEMENT_APP(cubesTabApp)
