#include "MainWindow.h"

#include <wx/artprov.h>
#include <wx/splitter.h>

#include "images/rocket16x16.xpm"
#include "images/rocket32x32.xpm"

namespace gui
{

MainWindowFrame::MainWindowFrame()
   : wxFrame(nullptr, wxID_ANY, "wxRocket", wxDefaultPosition, wxDefaultSize)
{
   // Set the icon
   wxIcon appIcon(rocket32x32);
   wxMask* appIconMask = new wxMask(appIcon, wxColor(0, 0, 0));
   appIcon.SetMask(appIconMask);
   this->SetIcon(appIcon);
   // Add a File menu
   wxMenu* fileMenu = new wxMenu;
   fileMenu->Append(wxID_NEW);
   fileMenu->AppendSeparator();
   fileMenu->Append(wxID_EXIT);

   // Add an Edit menu
   wxMenu* editMenu = new wxMenu;
   // Add a Tools menu
   wxMenu* toolsMenu = new wxMenu;
   wxBitmap launchItemIcon = wxBitmap(rocket16x16);
   wxMask* liMask = new wxMask(launchItemIcon, wxColor(0, 0, 0));
   launchItemIcon.SetMask(liMask);
   wxMenuItem* launchItem = toolsMenu->Append(wxID_ANY, _("&Launch"));

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
   toolbar->AddTool(launchItem->GetId(), _("Launch Rocket"), launchItemIcon);
   toolbar->Realize();

   // Create the layout

   // Create the splitter window, then we can add panels to it
   wxSplitterWindow* mainSplitter = new wxSplitterWindow(this,
                                                         wxID_ANY,
                                                         wxDefaultPosition,
                                                         wxDefaultSize,
                                                         wxSP_BORDER | wxSP_LIVE_UPDATE);
   wxSplitterWindow* designSplitter = new wxSplitterWindow(mainSplitter,
                                                           wxID_ANY,
                                                           wxDefaultPosition,
                                                           wxDefaultSize,
                                                           wxSP_BORDER | wxSP_LIVE_UPDATE);
   designSplitter->SetMinimumPaneSize(200);


   // NOTE, we never call delete manually on the panel since setting the parent pointer (this)
   // will put the onus on the parent to manage the resource. It'll get deleted
   // when the parent gets deleted. In this case the MainWindowFrame
   wxPanel* rocketTreePanel = new wxPanel(designSplitter,
                                          wxID_ANY,
                                          wxDefaultPosition,
                                          wxSize(200, 100));
   rocketTreePanel->SetBackgroundColour(wxColor(100, 100, 200));

   wxPanel* rocketComponentPanel = new wxPanel(designSplitter,
                                               wxID_ANY,
                                               wxDefaultPosition,
                                               wxSize(200, 100));
   rocketComponentPanel->SetBackgroundColour(wxColor(200, 100, 100));

   designSplitter->SplitVertically(rocketTreePanel, rocketComponentPanel);

   wxPanel* rocketVisPanel = new wxPanel(mainSplitter, wxID_ANY);
   rocketVisPanel->SetBackgroundColour(wxColor(100, 200, 100));

   mainSplitter->SplitHorizontally(designSplitter, rocketVisPanel);

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