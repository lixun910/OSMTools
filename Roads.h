//
// Created by Xun Li on 12/20/18.
//

#ifndef OSMTOOLSPROJECT_ROADS_H
#define OSMTOOLSPROJECT_ROADS_H

#include <vector>
#include <string>
// using boost unordered_map to support C++98 used in GeoDa
#include <boost/unordered_map.hpp>

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

        std::string GetValueFromLine(std::string line, bool quoted=true);

    protected:
        std::string base_url;
        // properties for nodes
        boost::unordered_map<std::string, int> id_map;
        std::vector<double> lat_arr;
        std::vector<double> lon_arr;
        std::vector<std::string> speed_arr;
        // properties for edges
        std::vector<std::string> edge_arr;
        std::vector<std::vector<std::string> > edges;
        std::vector<std::vector<std::string> > edge_properties;
    };
}


#endif //OSMTOOLSPROJECT_ROADS_H
