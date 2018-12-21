//
// Created by Xun Li on 12/20/18.
//

#ifndef OSMTOOLSPROJECT_ROADS_H
#define OSMTOOLSPROJECT_ROADS_H

#include <vector>
#include <string>
#include <unordered_map>

namespace OSMTools {
    enum RoadType {drive, walk};

    class Roads {
    public:
        Roads();
        ~Roads();

        //bool Download(const char* file_name);

        bool DownloadByBoundingBox(double lat_min, double lat_max,
                double lng_min, double lng_max, RoadType road_type,
                const char* file_name);

        //bool DownloadByPlaceNames();

        //bool DownloadByPolygon();

        //bool DownloadByRadius();
        void ReadOSMNodes(const char* file_name);

    protected:
        std::string GetOSMFilter(RoadType road_type);

        void SaveOSMToShapefile(const char* file_name);


    protected:
        std::string base_url;

        std::unordered_map<std::string, int> id_map;
        std::vector<double> lat_arr;
        std::vector<double> lon_arr;
        std::vector<std::string> speed_arr;

        std::unordered_map<std::string, int> edge_map;
        std::vector<std::string> highway_arr;
        std::unordered_map<std::string, std::vector<std::string> > edges;
    };
}


#endif //OSMTOOLSPROJECT_ROADS_H
