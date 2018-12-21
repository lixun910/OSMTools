//
// Created by Xun Li on 12/20/18.
//

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "Roads.h"

int main(void)
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
    //        "/Users/xun/Desktop/drive.json");
    roads.ReadOSMNodes("/Users/xun/Desktop/drive1.json");
    return 0;
}