//
// Created by Xun Li on 12/22/18.
//

#ifndef OSMTOOLSPROJECT_TRAVELTOOL_H
#define OSMTOOLSPROJECT_TRAVELTOOL_H

#include <vector>
#include <map>
#include <boost/unordered_map.hpp>
#include <wx/wx.h>

#include "../kNN/ANN/ANN.h"
#include <ogrsf_frmts.h>

#include "oclDijkstraKernel.h"
#include "Roads.h"

namespace OSMTools {
    typedef std::pair<double, double> RD_POINT;

    class TravelTool {
    public:
        TravelTool();

        TravelTool(std::vector<OGRFeature*> roads,
                std::vector<OGRFeature*> query_points);

        ~TravelTool();

    protected:

        void PreprocessRoads();

        bool MergeTwoWaysByStart(int w1, int w2);

        bool MergeTwoWaysByEnd(int w1, int w2);

        double ComputeArcDist(OGRPoint& from, OGRPoint& to);

        void AddEdge(int way_idx, OGRPoint& from,
                OGRPoint& to, double cost);

        void SaveMergedRoads(const char* shp_file_name);

        void SaveGraphToShapefile(const char* shp_file_name);
        
        void SaveQueryNodes(const char* shp_file_name);
        
        int GetValidEdgeId(int idx);
        
        wxString GetExeDir();

        bool SaveQueryResults(const char* file_path,
                              size_t num_nodes, int* results,
            const std::vector<std::pair<int, int> >& query_to_node);
    
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

        std::vector<bool> oneway_dict;

        // removed edge: concat to edge
        boost::unordered_map<int, int> removed_edges;

        boost::unordered_map<int,
            std::vector<std::pair<int, double> > > edges_dict;

        std::vector<OGRPoint> nodes;

        std::vector<bool> nodes_flag;

        boost::unordered_map<RD_POINT, int> nodes_dict;

        boost::unordered_map<std::pair<int, int>, double> pair_cost;

        // node: count of edge appearance
        boost::unordered_map<int, int> node_appearance;

        // end node: [way index, way index...]
        boost::unordered_map<int, std::vector<int> > endpoint_dict;

        // anchor points in roads
        boost::unordered_map<int, bool> anchor_points;

        // anchor point idx : idx-in-query_nodes
        boost::unordered_map<int, int> source_dict;

        // final query nodes for dijkstra
        std::vector<int> query_nodes;


        boost::unordered_map<std::string, int> speed_limit_dict;

    };

}

#endif //OSMTOOLSPROJECT_TRAVELTOOL_H
