#include <iostream>

#include <wx/wxprec.h>

#ifndef WX_PRECOMP
   #include <wx/wx.h>
#endif

#include "MainWindow.h"

class wxRocket : public wxApp
{
public:
   //wxRocket();
   //virtual ~wxRocket();

   virtual bool OnInit();

};

bool wxRocket::OnInit()
{
   gui::MainWindowFrame* frame = new gui::MainWindowFrame;
   frame->Show(true);
   return true;
}

// This will build the main function
wxIMPLEMENT_APP(wxRocket);