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
    enum RoadType {drive, walk, bike, transit};

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

        void SaveToShapefile(const char* file_name);

        void SaveEdgesToShapefile(const char* file_name);

        void SaveCSVToShapefile(const char* file_name);

        int GetNumNodes();

        int GetNumEdges(std::vector<std::string>& nodes);

        boost::unordered_map<std::string, int> id_map;
        std::vector<double> lat_arr;
        std::vector<double> lon_arr;
        boost::unordered_map<std::string,
            std::vector<std::pair<std::string, double> > > edge_dict;

    protected:
        std::string GetOSMFilter(RoadType road_type);

        std::string GetValueFromLine(std::string line, bool quoted=true);

        double ComputeArcDist(int from, int to);

    protected:
        std::string base_url;
        // properties for nodes


        // properties for edges
        std::vector<std::string> way_arr;
        boost::unordered_map<std::string, int> node_cnt_dict;
        std::vector<std::vector<std::string> > ways;
        // highway, name, county, oneway, maxspeed
        std::vector<std::vector<std::string> > way_properties;
    };
}


#endif //OSMTOOLSPROJECT_ROADS_H
