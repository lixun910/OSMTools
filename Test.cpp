//
// Created by Xun Li on 12/20/18.
//

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "Test.h"
#include "Roads.h"
#include "TravelTool.h"
#include "uiRoadDownload.h"

using namespace OSMTools;

int test(void)
{
    //OSMTools::Downloader downer;
    //downer.Get("https://github.com/GeoDaCenter/geoda/releases/download/1.12.1.183/GeoDa1.12.1.183-Installer.dmg",
    //        0, "/Users/xun/Desktop/GeoDa.dmg");
    double lat_min = 41.59466790;
    double lng_min = -87.98784460;
    double lat_max = 42.07277660;
    double lng_max = -87.47480050;
    OSMTools::Roads roads;
    //roads.DownloadByBoundingBox(lat_min, lng_min, lat_max, lng_max,
    //        OSMTools::drive,
    //        "/Users/xun/Dropbox/drive.json");
    roads.ReadOSMNodes("/Users/xun/Dropbox/drive.json");
    roads.SaveEdgesToShapefile("/Users/xun/Dropbox/drive.json");
    //roads.SaveCSVToShapefile("/Users/xun/Downloads/road/bld/_edges.csv");
    OSMTools::TravelTool tt(&roads);
    tt.QueryByCSV("/Users/xun/Dropbox/LEHD_blocks.csv");
    return 0;
}


class MyApp: public wxApp
{
public:
    virtual bool OnInit();
};
class MyFrame: public wxFrame
{
public:
    MyFrame(const wxString& title, const wxPoint& pos, const wxSize& size);
private:
    void OnHello(wxCommandEvent& event);
    void OnExit(wxCommandEvent& event);
    void OnAbout(wxCommandEvent& event);
wxDECLARE_EVENT_TABLE();
};
enum
{
    ID_Hello = 1
};
wxBEGIN_EVENT_TABLE(MyFrame, wxFrame)
                EVT_MENU(ID_Hello,   MyFrame::OnHello)
                EVT_MENU(wxID_EXIT,  MyFrame::OnExit)
                EVT_MENU(wxID_ABOUT, MyFrame::OnAbout)
wxEND_EVENT_TABLE()
wxIMPLEMENT_APP(MyApp);
bool MyApp::OnInit()
{
    MyFrame *frame = new MyFrame( "Hello World", wxPoint(50, 50), wxSize(450, 340) );
    frame->Show( true );
    return true;
}
MyFrame::MyFrame(const wxString& title, const wxPoint& pos, const wxSize& size)
        : wxFrame(NULL, wxID_ANY, title, pos, size)
{
    wxMenu *menuFile = new wxMenu;
    menuFile->Append(ID_Hello, "&Hello...\tCtrl-H",
                     "Help string shown in status bar for this menu item");
    menuFile->AppendSeparator();
    menuFile->Append(wxID_EXIT);
    wxMenu *menuHelp = new wxMenu;
    menuHelp->Append(wxID_ABOUT);
    wxMenuBar *menuBar = new wxMenuBar;
    menuBar->Append( menuFile, "&File" );
    menuBar->Append( menuHelp, "&Help" );
    SetMenuBar( menuBar );
    CreateStatusBar();
    SetStatusText( "Welcome to wxWidgets!" );
}
void MyFrame::OnExit(wxCommandEvent& event)
{
    Close( true );
}
void MyFrame::OnAbout(wxCommandEvent& event)
{
    wxMessageBox( "This is a wxWidgets' Hello world sample",
                  "About Hello World", wxOK | wxICON_INFORMATION );
}
void MyFrame::OnHello(wxCommandEvent& event)
{
    double lat_min = 41.59466790;
    double lng_min = -87.98784460;
    double lat_max = 42.07277660;
    double lng_max = -87.47480050;
    OSMTools::Roads roads;
    //roads.DownloadByBoundingBox(lat_min, lng_min, lat_max, lng_max, OSMTools::drive, "/Users/xun/Dropbox/drive.json");
    roads.ReadOSMNodes("/Users/xun/Dropbox/drive.json");
    roads.SaveEdgesToShapefile("/Users/xun/Dropbox/drive.json");
    //roads.SaveCSVToShapefile("/Users/xun/Downloads/road/bld/_edges.csv");
    OSMTools::TravelTool tt(&roads);
    tt.QueryByCSV("/Users/xun/Dropbox/LEHD_blocks.csv");
}

