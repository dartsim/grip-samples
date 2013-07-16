/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * 
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file SimulationHistoryTab.cpp
 * @brief Maintains a log of simulation start and stop events
 * @author Saul Reynolds-Haertle <saulrh@gatech.edu>
 * @date 2013/01/15
 */

#include "SimulationHistoryTab.h"

#include <wx/wx.h>

#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <iostream>
using namespace std;

#include <Tabs/AllTabs.h>
#include <GRIPApp.h>



/** Events */
enum SimulationHistoryTabIDs {
    id_listcontrol_state = 927,
    id_button_loadsnapshot,
    id_button_delsnapshot,
};

/** Handler for events **/
BEGIN_EVENT_TABLE(SimulationHistoryTab, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, SimulationHistoryTab::OnButton)
EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, SimulationHistoryTab::OnSlider)
END_EVENT_TABLE()


// Class constructor for the tab: Each tab will be a subclass of GRIPTab
IMPLEMENT_DYNAMIC_CLASS(SimulationHistoryTab, GRIPTab)

/**
 * @function SimulationHistoryTab
 * @brief Constructor
 */
SimulationHistoryTab::SimulationHistoryTab(wxWindow *parent, const wxWindowID id,
                                           const wxPoint& pos, const wxSize& size, long style) :
GRIPTab(parent, id, pos, size, style) {
    sizerFull = new wxBoxSizer(wxHORIZONTAL);
  
    // Create Static boxes (outline of your Tab)
    wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("Simulation Log"));
    wxStaticBox* ss2Box = new wxStaticBox(this, -1, wxT("Time Travel"));
  
    // Create sizers for these static boxes
    wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);
    wxStaticBoxSizer* ss2BoxS = new wxStaticBoxSizer(ss2Box, wxVERTICAL);

    // Create some buttons
    ss2BoxS->Add(new wxButton(this, id_button_loadsnapshot, wxT("Load Selected")), 0, wxALL, 1);
    ss2BoxS->Add(new wxButton(this, id_button_delsnapshot, wxT("Delete Selected")), 0, wxALL, 1);

    // Create and set up the listcontrol for displaying snapshots
    stateListControl = new wxListCtrl(this,
                                      id_listcontrol_state,
                                      wxDefaultPosition,
                                      wxDefaultSize,
                                      wxLC_REPORT | wxLC_SINGLE_SEL);

    wxListItem itemCol;
    itemCol.SetText(wxT("Time"));
    itemCol.SetImage(-1);
    stateListControl->InsertColumn(0, itemCol);
    itemCol.SetText(wxT("Idx"));
    itemCol.SetImage(-1);
    stateListControl->InsertColumn(1, itemCol);

    stateListControl->SetColumnWidth(0, 300);
    stateListControl->SetColumnWidth(1, wxLIST_AUTOSIZE_USEHEADER);

    ss1BoxS->Add(stateListControl, 1, wxEXPAND | wxALL, 1);

    // Add the boxes to their respective sizers
    sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 1);
    sizerFull->Add(ss2BoxS, 1, wxEXPAND | wxALL, 1);
    SetSizer(sizerFull);
}


/**
 * @function OnButton
 * @brief Handles button events
 */
void SimulationHistoryTab::OnButton(wxCommandEvent & _evt) {
    int slnum = _evt.GetId();
    long selectedIndex = -1;
    int savedIndex;
    int type = 0;
    
    switch(slnum) {
    case id_button_loadsnapshot: {
        selectedIndex = stateListControl->GetNextItem(selectedIndex, wxLIST_NEXT_ALL, wxLIST_STATE_SELECTED);
        if (selectedIndex == -1)
        {
            std::cout << "Nothing selected" << std::endl;
            return;
        }
        savedIndex = savedIndices[selectedIndex];
        mWorld->setTime(savedTimes[savedIndex]);
        mWorld->setState(savedStates[savedIndex]);
        // and then ask the frame to redraw the display
        wxCommandEvent evt(wxEVT_GRIP_SIMULATE_FRAME,GetId());
        evt.SetEventObject(this);
        evt.SetClientData((void*)&type);
        GetEventHandler()->AddPendingEvent(evt);
        break; }
    case id_button_delsnapshot: {
        selectedIndex = stateListControl->GetNextItem(selectedIndex, wxLIST_NEXT_ALL, wxLIST_STATE_SELECTED);
        if (selectedIndex == -1)
        {
            std::cout << "Nothing selected" << std::endl;
            return;
        }
        stateListControl->DeleteItem(selectedIndex);
        break; }
    default: {
        printf("Default button \n");
        break; }
    }
}

void SimulationHistoryTab::OnSlider(wxCommandEvent &evt) {
}

void SimulationHistoryTab::GRIPStateChange() {
}

void SimulationHistoryTab::GRIPEventSceneLoaded() {
}

void SimulationHistoryTab::GRIPEventSceneUnloaded() {
    stateListControl->DeleteAllItems();
    savedStates.clear();
    savedTimes.clear();
    savedIndices.clear();
}

void SimulationHistoryTab::GRIPEventSimulationStart() {
    takeSnapshot();
}
void SimulationHistoryTab::GRIPEventSimulationStop() {
    takeSnapshot();
}

void SimulationHistoryTab::takeSnapshot(){
    static int snapshotCount = 0;

    // add a corresponding item to the list control
    wxString buf;
    buf.Format(wxT("Snapshot %d, T = %f"), snapshotCount, mWorld->getTime());
    long insertedAt = stateListControl->InsertItem(snapshotCount, buf, 0);
    stateListControl->SetItemData(insertedAt, snapshotCount);
    buf.Printf(_T("%f"), mWorld->getTime());
    stateListControl->SetItem(insertedAt, 0, buf);
    buf.Printf(_T("%d"), snapshotCount);
    stateListControl->SetItem(insertedAt, 1, buf);

    // save a world state and update data structures
    savedStates.push_back(mWorld->getState());
    savedTimes.push_back(mWorld->getTime());
    assert(savedStates.size() == savedTimes.size());
    savedIndices.insert(std::make_pair<int, int>(insertedAt, savedStates.size() - 1));
    
    // increment counter
    snapshotCount++;
}
