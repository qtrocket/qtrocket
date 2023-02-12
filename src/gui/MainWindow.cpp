#include "MainWindow.h"

#include <wx/artprov.h>
#include <wx/splitter.h>

namespace gui
{

MainWindowFrame::MainWindowFrame()
   : wxFrame(nullptr, wxID_ANY, "wxRocket", wxDefaultPosition, wxDefaultSize)
{
   // Add a File menu
   wxMenu* fileMenu = new wxMenu;
   fileMenu->Append(wxID_NEW);
   fileMenu->AppendSeparator();
   fileMenu->Append(wxID_EXIT);

   // Add an Edit menu
   wxMenu* editMenu = new wxMenu;
   // Add a Tools menu
   wxMenu* toolsMenu = new wxMenu;

   // Add a Help menu
   wxMenu* helpMenu = new wxMenu;
   helpMenu->Append(wxID_ABOUT);

   wxMenuBar* menuBar = new wxMenuBar;
   menuBar->Append(fileMenu, _("&File"));
   menuBar->Append(editMenu, _("&Edit"));
   menuBar->Append(toolsMenu, _("&Tools"));
   menuBar->Append(helpMenu, _("&Help"));

   SetMenuBar(menuBar);
   CreateStatusBar();
   SetStatusText(_("Welcome to wxRocket!"));

   wxToolBar* toolbar = CreateToolBar();
   toolbar->AddTool(wxID_NEW, _("New"), wxArtProvider::GetBitmap("wxART_NEW"));
   toolbar->Realize();

   // Create the layout

   // Create the splitter window, then we can add panels to it
   wxSplitterWindow* windowSplitter = new wxSplitterWindow(this,
                                                           wxID_ANY,
                                                           wxDefaultPosition,
                                                           wxDefaultSize,
                                                           wxSP_BORDER | wxSP_LIVE_UPDATE);
   windowSplitter->SetMinimumPaneSize(200);
   //wxSplitterWindow* topSplitter    = new wxSplitterWindow(this);


   // NOTE, we never call delete manually on the panel since setting the parent pointer (this)
   // will put the onus on the parent to manage the resource. It'll get deleted
   // when the parent gets deleted. In this case the MainWindowFrame
   wxPanel* rocketTreePanel = new wxPanel(windowSplitter,
                                          wxID_ANY,
                                          wxDefaultPosition,
                                          wxSize(200, 100));
   rocketTreePanel->SetBackgroundColour(wxColor(100, 100, 200));

   wxPanel* rocketComponentPanel = new wxPanel(windowSplitter,
                                               wxID_ANY,
                                               wxDefaultPosition,
                                               wxSize(200, 100));
   rocketComponentPanel->SetBackgroundColour(wxColor(200, 100, 100));

   windowSplitter->SplitVertically(rocketTreePanel, rocketComponentPanel);
/*
   wxSizer* rocketDesignSizer = new wxBoxSizer(wxHORIZONTAL);
   rocketDesignSizer->Add(rocketTreePanel, 1, wxEXPAND | wxRIGHT, 2);
   rocketDesignSizer->Add(rocketComponentPanel, 1, wxEXPAND);
*/

/*
   wxPanel* rocketVisPanel = new wxPanel(this, wxID_ANY);
   rocketVisPanel->SetBackgroundColour(wxColor(100, 200, 100));
*/

//   wxBoxSizer* mainSizer = new wxBoxSizer(wxVERTICAL);
//   mainSizer->Add(rocketDesignSizer, 2, wxEXPAND | wxALL, 2);
//   mainSizer->Add(rocketVisPanel, 1, wxEXPAND | wxLEFT | wxBOTTOM | wxRIGHT, 2);
//   this->SetSizerAndFit(mainSizer);
   /*
   SetSizer(mainSizer);
   wxTextCtrl* firstNameBox = new wxTextCtrl(this, wxID_ANY);
   mainSizer->Add(firstNameBox, 1, wxEXPAND);
   */


   Bind(wxEVT_MENU, &MainWindowFrame::onAbout, this, wxID_ABOUT);
   Bind(wxEVT_MENU, &MainWindowFrame::onExit, this, wxID_EXIT);

}

void MainWindowFrame::onExit(wxCommandEvent& evt)
{
   Close(true);
}

void MainWindowFrame::onAbout(wxCommandEvent& evt)
{
   wxMessageBox(_("This is wxRocket. (c) 2023 by Travis Hunter"),
                _("About wxRocket"), wxOK | wxICON_INFORMATION);
}

}