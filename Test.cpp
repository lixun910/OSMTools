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



wxIMPLEMENT_APP(TestApp);
bool TestApp::OnInit()
{
    double lat_min = 41.59466790;
    double lng_min = -87.98784460;
    double lat_max = 42.07277660;
    double lng_max = -87.47480050;

    uiRoadDownload *road_frame = new uiRoadDownload(lng_max, lng_min,
            lat_min, lat_max);
    road_frame->Show( true );
    return true;
}

