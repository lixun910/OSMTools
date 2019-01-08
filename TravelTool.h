//
// Created by Xun Li on 12/22/18.
//

#ifndef OSMTOOLSPROJECT_TRAVELTOOL_H
#define OSMTOOLSPROJECT_TRAVELTOOL_H

#include <vector>
#include <map>
#include <boost/unordered_map.hpp>
#include <wx/wx.h>

#ifdef __GEODA__
#include "../kNN/ANN/ANN.h"
#else
#include <ANN/ANN.h>
#endif
#include <ogrsf_frmts.h>

#include "oclDijkstraKernel.h"
#include "Roads.h"

namespace OSMTools {
    typedef std::pair<double, double> RD_POINT;

    class TravelTool {
    public:
        TravelTool();

        TravelTool(std::vector<OGRFeature*> roads);

        ~TravelTool();

        void GetDistanceMatrix(std::vector<OGRFeature*> query_points);

        void BuildCPUGraph();

        int Query(OGRPoint& from_pt, OGRPoint& to_pt,
                  std::vector<OGRLineString>& ogr_line,
                  std::vector<int>& way_ids);

        void QueryHexMap(OGRPoint& start_pt, OGREnvelope& extent,
                         double hexagon_radius,
                         std::vector<std::vector<OGRPolygon> >& hexagons,
                         std::vector<std::vector<int> >& costs,
                         bool create_hexagons = true);
    protected:

        void PreprocessRoads();
        
        void ComputeDistMatrix(int* results);
        
        void ComputeDistMatrixCPU(int* results, int query_size, int nCPUs);

        void ComputeDistMatrixGPU(int* results, int query_size, int offset);

        void dijkstra_thread(CPUGraph* graph,
                             const std::vector<int>& query_indexes,
                             int* results,
                             const std::vector<std::pair<int, int> >& query_to_node,
                             boost::unordered_map<int, std::vector<int> >& node_to_query,
                             int a, int b);
        
        bool MergeTwoWaysByStart(int w1, int w2);

        bool MergeTwoWaysByEnd(int w1, int w2);

        double EarthMeterToDegree(double d);
        
        double ComputeArcDist(OGRPoint& from, OGRPoint& to);

        void AddEdge(int way_idx, OGRPoint& from,
                OGRPoint& to, int cost);

        void SaveMergedRoads(const char* shp_file_name);

        void SaveGraphToShapefile(const char* shp_file_name);
        
        void SaveQueryNodes(const char* shp_file_name);
        
        int GetValidEdgeId(int idx);
        
        wxString GetExeDir();

        bool SaveQueryResults(const char* file_path,
                              int n_query, int* results,
                              const std::vector<wxString>& query_ids);
    
        int DetectGPU();
        
    protected:
        double** xy;
        ANNkd_tree* kd_tree;

        CPUGraph* cpu_graph;

        float ratio_cpu_to_gpu;
        int  num_gpus;
        int  num_cores;
        
        int  num_nodes;
        int  num_edges;

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

        // [node idx, node idx] : cost (seconds)
        boost::unordered_map<std::pair<int, int>, int> pair_cost;

        // [node idx, node idx] : way id
        boost::unordered_map<std::pair<int, int>, int> edge_to_way;

        // node: count of edge appearance
        boost::unordered_map<int, int> node_appearance;

        // end node: [way index, way index...]
        boost::unordered_map<int, std::vector<int> > endpoint_dict;

        // anchor points in roads
        boost::unordered_map<int, bool> anchor_points;

        // anchor point idx : [idx-in-query_nodes]
        boost::unordered_map<int, std::vector<int> > node_to_query;

        // idx-in-query : [anchor point idx, cost]
        std::vector<std::pair<int, int> > query_to_node;
        
        // final query nodes for dijkstra
        std::vector<int> query_nodes;


        boost::unordered_map<std::string, int> speed_limit_dict;

    };

}

#endif //OSMTOOLSPROJECT_TRAVELTOOL_H
