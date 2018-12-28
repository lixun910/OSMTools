//
// Created by Xun Li on 12/22/18.
//

#ifndef OSMTOOLSPROJECT_TRAVELTOOL_H
#define OSMTOOLSPROJECT_TRAVELTOOL_H

#include <vector>
#include <boost/unordered_map.hpp>
#include <ANN/ANN.h>
#include <ogrsf_frmts.h>

#include "oclDijkstraKernel.h"
#include "Roads.h"

namespace OSMTools {
    typedef std::pair<std::string, double> nbr_cost;
    typedef std::pair<double, double> RD_POINT;

    class TravelTool {
    public:
        TravelTool();

        TravelTool(Roads* roads);

        TravelTool(const char* road_shp_path,
                const char* query_pts_shp_path);

        TravelTool(std::vector<OGRFeature*> roads,
                std::vector<OGRFeature*> query_points);

        ~TravelTool();

        double* QueryByCSV(const char* file_path);

    protected:

        void BuildKdTreeFromRoads();

        void MergeTwoWaysByStart(int w1, int w2);

        void MergeTwoWaysByEnd(int w1, int w2);

        void BuildKdTree();

        void InitFromCSV(const char* file_name);

        void InitFromTable(std::vector<std::string>& from,
                std::vector<std::string>& to,
                std::vector<std::string>& highway,
                std::vector<std::string>& maxspeed,
                std::vector<std::string>& oneway,
                std::vector<double>& distance);

    protected:
        GraphData graph;
        ANNkd_tree* kd_tree;

        int  num_nodes;
        int  num_edges;
        int* vertex_array;
        int* edge_array;
        int* weight_array;
        double** query_xy;

        std::vector<OGRFeature*> roads;
        std::vector<OGRFeature*> query_points;


        std::vector<std::vector<OGRPoint> > edges;

        std::vector<RD_POINT> node_array;

        boost::unordered_map<int, bool> removed_edges;

        // node: count of edge appearance
        boost::unordered_map<RD_POINT, int> node_appearance;

        // end node: [way index, way index...]
        boost::unordered_map<RD_POINT, std::vector<int> > endpoint_dict;

        // anchor points in roads
        boost::unordered_map<RD_POINT, bool> anchor_points;

        // anchor point idx : <query point idx, distance>
        boost::unordered_map<int,
            std::vector<std::pair<int, double> > > source_dict;

        // final query nodes for dijkstra
        std::vector<int> query_nodes;




        // node_id array
        std::vector<std::string> node_ids;

        // node_id_name : index
        boost::unordered_map<std::string, int> node_id_dict;
        // node_id_name : [(node_id_name, dist), (node_id_name, dist), ...]
        boost::unordered_map<std::string, std::vector<nbr_cost> > edge_dict;

        boost::unordered_map<std::string, int> speed_limit_dict;
    };

}

#endif //OSMTOOLSPROJECT_TRAVELTOOL_H
