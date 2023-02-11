#include <iostream>

#include <wx/wxprec.h>

#ifndef WX_PRECOMP
   #include <wx/wx.h>
#endif

class wxRocket : public wxApp
{
public:
   //wxRocket();
   //virtual ~wxRocket();

   virtual bool OnInit();

};

class MainWindowFrame : public wxFrame
{
public:
   MainWindowFrame();

private:
   void onExit(wxCommandEvent& evt);
   void onAbout(wxCommandEvent& evt);

};


bool wxRocket::OnInit()
{
   MainWindowFrame* frame = new MainWindowFrame;
   frame->Show(true);
   return true;
}

MainWindowFrame::MainWindowFrame()
   : wxFrame(nullptr, wxID_ANY, "wxRocket")
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

// This will build the main function
wxIMPLEMENT_APP(wxRocket);