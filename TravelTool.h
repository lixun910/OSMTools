//
// Created by Xun Li on 12/22/18.
//

#ifndef OSMTOOLSPROJECT_TRAVELTOOL_H
#define OSMTOOLSPROJECT_TRAVELTOOL_H

#include <vector>
#include <boost/unordered_map.hpp>
#include <ANN/ANN.h>

#include "oclDijkstraKernel.h"
#include "Roads.h"

namespace OSMTools {
    typedef std::pair<std::string, double> nbr_cost;

    class TravelTool {
    public:
        TravelTool();

        TravelTool(Roads* roads);

        //TravelTool(std::vector<)

        ~TravelTool();

        double* Query(std::vector<double> lats, std::vector<double> lons);

        double* QueryByCSV(const char* file_path);

    protected:
        void InitCPUGPU();

        void BuildKdTree(int node_cnt, double** xy);

        void ComputeDistanceMatrix();

        void GetTravelRoute(double from_lat, double from_lon, double to_lat, double to_lon);

        void InitFromCSV(const char* file_name);

        void InitFromTable(std::vector<std::string>& from, std::vector<std::string>& to,
                           std::vector<std::string>& highway, std::vector<std::string>& maxspeed,
                           std::vector<std::string>& oneway, std::vector<double>& distance);

    protected:
        GraphData graph;
        ANNkd_tree* kd_tree;

        int num_nodes;
        int num_edges;
        int* vertex_array;
        int* edge_array;
        int* weight_array;


        bool has_gpus;
        bool has_cpu;

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
