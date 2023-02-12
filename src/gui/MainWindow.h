#ifndef GUI_MAINWINDOW_H
#define GUI_MAINWINDOW_H

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
   #include <wx/wx.h>
#endif

namespace gui
{

class MainWindowFrame : public wxFrame
{
public:
   MainWindowFrame();

private:
   void onExit(wxCommandEvent& evt);
   void onAbout(wxCommandEvent& evt);

};


}

#endif // GUI_MAINWINDOW_H