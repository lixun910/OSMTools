//
// Created by Xun Li on 12/22/18.
//
#include <boost/date_time/posix_time/posix_time.hpp>
#include "csv.h"  // https://github.com/ben-strasser/fast-cpp-csv-parser

#include "TravelTool.h"

using namespace OSMTools;
namespace pt = boost::posix_time;

//boost::unordered_map<std::string, int> TravelTool::speed_limit_dict

TravelTool::TravelTool()
{
    speed_limit_dict = {
            {"road",32}, {"motorway", 96},
            {"motorway_link", 48},
            {"motorway_junction", 48},
            {"trunk", 80},
            {"trunk_link", 40},
            {"primary", 48},
            {"primary_link", 32},
            {"secondary", 40},
            {"secondary_link", 40},
            {"tertiary", 32},
            {"tertiary_link", 32},
            {"residential", 24},
            {"living_street", 16},
            {"service", 16},
            {"track", 32},
            {"pedestrian", 3.2},
            {"services", 3.2},
            {"bus_guideway", 3.2},
            {"path", 8},
            {"cycleway", 16},
            {"footway", 3.2},
            {"bridleway", 3.2},
            {"byway", 3.2},
            {"steps", 0.16},
            {"unclassified", 24},
            {"lane", 16},
            {"opposite_lane", 16},
            {"opposite", 16},
            {"grade1", 16}, {"grade2", 16}, {"grade3", 16}, {"grade4", 16}, {"grade5", 16}, {"roundabout", 40}
    };
}

TravelTool::TravelTool(Roads* roads)
: TravelTool()
{
    num_edges = roads->GetNumEdges(node_ids);
    num_nodes = node_ids.size();

    for (unsigned int i=0; i<num_nodes; ++i) {
        node_id_dict[node_ids[i]] = i;
    }

    vertex_array = (int*) malloc(num_nodes * sizeof(int));
    edge_array = (int*)malloc(num_edges * sizeof(int));
    weight_array = (int*)malloc(num_edges * sizeof(int));

    graph.vertexCount = num_nodes;
    graph.vertexArray = vertex_array;
    graph.edgeCount = num_edges;
    graph.edgeArray = edge_array;
    graph.weightArray = weight_array;

    size_t offset = 0, e_idx = 0, e_size;
    double cost = 0;
    for(unsigned int i = 0; i < graph.vertexCount; i++) {
        graph.vertexArray[i] = offset;
        std::string node = node_ids[i];
        if (roads->edge_dict.find(node) == roads->edge_dict.end())
            continue;
        e_size = roads->edge_dict[node].size();
        for (unsigned int j=0; j<e_size; j++) {
            std::string nbr_id = roads->edge_dict[node][j].first;
            cost = roads->edge_dict[node][j].second;
            graph.edgeArray[e_idx] = node_id_dict[nbr_id];
            graph.weightArray[e_idx++] = cost;
        }
        offset += e_size;
    }

    double** xy = new double*[num_nodes];
    for (size_t i=0; i<num_nodes; i++) {
        std::string node = node_ids[i];
        int idx = roads->id_map[node];
        xy[i] = new double[2];
        xy[i][0] = roads->lon_arr[idx];
        xy[i][1] = roads->lat_arr[idx];
    }
    kd_tree = new ANNkd_tree(xy, num_nodes, 2);
}

TravelTool::TravelTool(const char* road_shp_path,
                       const char* query_pts_shp_path)
{

}

TravelTool::TravelTool(std::vector<OGRFeature*> in_roads,
        std::vector<OGRFeature*> in_query_points)
    : TravelTool()
{
    roads = in_roads;
    query_points = in_query_points;

    BuildKdTreeFromRoads();
}

TravelTool::~TravelTool()
{
    if (query_xy) {
        for (size_t i=0; i<query_points.size(); ++i) {
            delete[] query_xy[i];
        }
        delete[] query_xy;
    }
    free(vertex_array);
    free(edge_array);
    free(weight_array);

    if (kd_tree) delete kd_tree;
}

void TravelTool::BuildKdTreeFromRoads()
{
    if (roads.empty() == true) return;

    size_t n_roads = roads.size();
    OGRFeature* feature;
    OGRGeometry* geom;
    OGRLineString* line;
    int i, j, n_pts;

    for (i=0; i<n_roads; ++i) {
        feature = roads[i];
        geom = feature->GetGeometryRef();
        std::vector<OGRPoint> e;
        if (geom && geom->IsEmpty() == false) {
            line = (OGRLineString*) geom;
            n_pts = line->getNumPoints();
            for (j=0; j<n_pts; ++j) {
                OGRPoint pt;
                line->getPoint(j, &pt);
                RD_POINT rd_pt =
                        std::make_pair(pt.getX(), pt.getY());
                if (node_appearance.find(rd_pt) !=
                        node_appearance.end()) {
                    node_appearance[rd_pt] = 1;
                } else {
                    node_appearance[rd_pt] += 1;
                }
                // end points
                if (j==0 || j == n_pts-1) {
                    endpoint_dict[rd_pt].push_back(i);
                }
                e.push_back(pt); // todo: possible memory issue
            }
        }
    }

    int n_nodes = node_appearance.size();
    double** xy = new double*[n_nodes];
    i = 0;
    boost::unordered_map<RD_POINT, int>::iterator it;
    for (it=node_appearance.begin();
            it != node_appearance.end(); ++it) {
        RD_POINT pt = it->first;
        node_array.push_back(pt);
        xy[i] = new double[2];
        xy[i][0] = pt.first;
        xy[i][1] = pt.second;
        i++;
    }
    kd_tree = new ANNkd_tree(xy, n_nodes, 2);

    // using query points to find anchor points from roads
    double eps = 0; // error bound
    ANNidxArray nnIdx = new ANNidx[1];
    ANNdistArray dists = new ANNdist[1];
    int query_size = query_points.size(), q_id;
    for (size_t i=0; i<query_size; ++i) {
        feature = query_points[i];
        geom = feature->GetGeometryRef();
        OGRPoint* m_pt = (OGRPoint*)geom;
        double* q_pt = new double[2];
        q_pt[0] = m_pt->getY();
        q_pt[1] = m_pt->getX();
        kd_tree->annkSearch(q_pt, 1, nnIdx, dists, eps);
        delete[] q_pt;
        q_id = nnIdx[0];
        anchor_points[ node_array[q_id] ] = true;
        if (source_dict.find( q_id ) == source_dict.end()) {
            query_nodes.push_back(q_id);
        }
        source_dict[q_id].push_back(
                std::make_pair(i, dists[0]/20.0) );
    }
    delete[] nnIdx;
    delete[] dists;
    delete kd_tree;

    // build graph
    // remove circle way
    for (i=0; i<edges.size(); ++i) {
        n_pts = edges[i].size();
        OGRPoint start = edges[i][0];
        OGRPoint end = edges[i][n_pts-1];
        if (start.Equals(&end)) {
            removed_edges[i] = true;
        }
    }
    // concat ways
    for (i=0; i<edges.size(); ++i) {
        if (removed_edges.find(i) != removed_edges.end()) {
            // this edge has been merged with other edge already
            // so skip
            continue;
        }
        n_pts = edges[i].size();
        OGRPoint start = edges[i][0];
        OGRPoint end = edges[i][n_pts-1];
        RD_POINT rd_start = std::make_pair(start.getX(), start.getY());
        std::vector<int> conn_ways;
        conn_ways = endpoint_dict[rd_start];
        bool is_merged = false;
        if (conn_ways.size() == 2) {
            int w1 = i;
            int w2 = conn_ways[0] == i ? conn_ways[1] : conn_ways[0];
            MergeTwoWaysByStart(w1, w2);
            removed_edges[w2] = true;
            is_merged = true;
        }
        RD_POINT rd_end = std::make_pair(end.getX(), end.getY());
        conn_ways = endpoint_dict[rd_end];
        if (conn_ways.size() == 2) {
            int w1 = i;
            int w2 = conn_ways[0] == i ? conn_ways[1] : conn_ways[0];
            MergeTwoWaysByEnd(w1, w2);
            removed_edges[w2] = true;
            is_merged = true;
        }
        if (is_merged) {
            i--; // reprocessing current way since it's merged
        }
    }
    // simplify ways
    for (i=0; i<edges.size(); ++i) {
        if (removed_edges.find(i) != removed_edges.end()) {
            continue;
        }
        n_pts = edges[i].size();
        double dist = 0;
        int from, to;
        for (j=edges[i].size()-2; j>=0; --j) {
            OGRPoint pt = edges[i][j];
            RD_POINT rd_pt = std::make_pair(pt.getX(), pt.getY());
            if (j> 0 && node_appearance[rd_pt] == 1 &&
                anchor_points.find(rd_pt) == anchor_points.end())
            {
                dist += GetArcDistance(pt, prev_pt);
                edges[i].erase(edges[i].begin() + j);
            } else {
                // add {from, to, dist} to edge_dict
                // edges[i][j] = from, edges[i][j+1] = to
                dist += GetArcDistance(pt, prev_pt);
                dist = 0; // reset dist to 0
            }
        }
    }

    vertex_array = (int*) malloc(num_nodes * sizeof(int));
    edge_array = (int*)malloc(num_edges * sizeof(int));
    weight_array = (int*)malloc(num_edges * sizeof(int));

    graph.vertexCount = num_nodes;
    graph.vertexArray = vertex_array;
    graph.edgeCount = num_edges;
    graph.edgeArray = edge_array;
    graph.weightArray = weight_array;

    size_t offset = 0, e_idx = 0, e_size;
    double cost = 0;
    for(unsigned int i = 0; i < graph.vertexCount; i++) {
        graph.vertexArray[i] = offset;
        std::string node = node_ids[i];
        if (roads->edge_dict.find(node) == roads->edge_dict.end())
            continue;
        e_size = roads->edge_dict[node].size();
        for (unsigned int j=0; j<e_size; j++) {
            std::string nbr_id = roads->edge_dict[node][j].first;
            cost = roads->edge_dict[node][j].second;
            graph.edgeArray[e_idx] = node_id_dict[nbr_id];
            graph.weightArray[e_idx++] = cost;
        }
        offset += e_size;
    }
}

void TravelTool::MergeTwoWaysByStart(int w1, int w2)
{
    std::vector<OGRPoint> e1 = edges[w1];
    std::vector<OGRPoint> e2 = edges[w2];
    OGRPoint start1 = e1[0];
    OGRPoint start2 = e2[0];
    OGRPoint end2 = e2[e2.size() -1];

    if (start1.Equals(&start2)) {
        // reverse w2, concat w1
        for (size_t i=0; i < e2.size(); ++i) {
            e1.insert(e1.begin(), e2[i]);
        }
    } else if (start1.Equals(&end2)) {
        // w2 concat w1
        for (size_t i=e2.size()-1; i >= 0; --i) {
            e1.insert(e1.begin(), e2[i]);
        }
    }
    RD_POINT rd_pt = std::make_pair(start1.getX(), start1.getY());
    node_appearance[rd_pt]-=1;
}

void TravelTool::MergeTwoWaysByEnd(int w1, int w2)
{
    std::vector<OGRPoint> e1 = edges[w1];
    std::vector<OGRPoint> e2 = edges[w2];
    OGRPoint end1 = e1[e1.size()-1];
    OGRPoint start2 = e2[0];
    OGRPoint end2 = e2[e2.size() -1];

    if (end1.Equals(&start2)) {
        // w1, concat w2
        for (size_t i=0; i < e2.size(); ++i) {
            e1.push_back(e2[i]);
        }
    } else if (end1.Equals(&end2)) {
        // w1 concat reversed w2
        for (size_t i=e2.size()-1; i >= 0; --i) {
            e1.push_back(e2[i]);
        }
    }
    RD_POINT rd_pt = std::make_pair(end1.getX(), end1.getY());
    node_appearance[rd_pt]-=1;
}

void TravelTool::BuildKdTree()
{
    if (query_points.empty() == true) return;

    int query_size = query_points.size();
    query_xy = new double*[query_size];

    OGRFeature* feature;
    OGRGeometry* geom;
    OGRPoint* pt;
    for (size_t i=0; i<query_size; ++i) {
        query_xy[i] = new double[2];
        feature = query_points[i];
        geom = feature->GetGeometryRef();
        if (geom && geom->IsEmpty() == false) {
            pt = (OGRPoint*)geom;
            query_xy[i][0] = pt->getX();
            query_xy[i][1] = pt->getY();
        }
    }
    if (kd_tree != 0) {
        delete kd_tree;
    }
    kd_tree = new ANNkd_tree(query_xy, query_size, 2);
}

void TravelTool::InitFromCSV(const char *file_name) {
    if (file_name == 0) return;

    io::CSVReader<6> in(file_name);
    in.read_header(io::ignore_extra_column, "distance", "from", "to", "highway", "maxspeed", "oneway");
    double distance; std::string from; std::string  to; std::string highway; std::string maxspeed; std::string oneway;
    int node_idx = 0;
    while(in.read_row(distance, from, to, highway, maxspeed, oneway)){
        if (distance <= 0) {
            continue;
        }
        if (node_id_dict.find(from) == node_id_dict.end()) {
            node_id_dict[from] = node_idx;
            node_idx ++;
            node_ids.push_back(from);
        }
        if (node_id_dict.find(to) == node_id_dict.end()) {
            node_id_dict[to] = node_idx;
            node_idx ++;
            node_ids.push_back(to);
        }

        double w = 0;
        if (maxspeed.empty()) {
            if (speed_limit_dict.find(highway) != speed_limit_dict.end()) {
                double sl = speed_limit_dict[highway];
                w = distance / sl;
            } else {
                w = distance / 40.0;  // for not labeld road
            }
        } else {
            std::string sp_str = maxspeed.substr(0,2);
            double sl = std::stoi(sp_str);
            w = distance / sl;
        }

        edge_dict[from].push_back(std::make_pair(to, w));
        if (oneway != "yes") {
            edge_dict[to].push_back(std::make_pair(from,w));
        }
    }

}

void TravelTool::InitFromTable(std::vector<std::string> &froms, std::vector<std::string> &tos,
                               std::vector<std::string> &highways, std::vector<std::string> &maxspeeds,
                               std::vector<std::string> &oneways, std::vector<double> &distances) {

    size_t n_obs = froms.size();
    size_t node_idx = 0;
    double distance, sl, w;
    std::string from, to, maxspeed, highway, oneway;

    for (size_t i= 0; i<n_obs; ++i) {
        distance = distances[i];
        from = froms[i];
        to = tos[i];
        maxspeed = maxspeeds[i];
        highway = highways[i];
        oneway = oneways[i];

        if (distance <= 0) continue;

        if (node_id_dict.find(from) == node_id_dict.end()) {
            node_id_dict[from] = node_idx;
            node_idx ++;
            node_ids.push_back(from);
        }
        if (node_id_dict.find(to) == node_id_dict.end()) {
            node_id_dict[to] = node_idx;
            node_idx ++;
            node_ids.push_back(to);
        }

        w = 0;
        if (maxspeed.empty()) {
            if (speed_limit_dict.find(highway) != speed_limit_dict.end()) {
                sl = speed_limit_dict[highway];
                w = distance / sl;
            } else {
                w = distance / 40.0;  // for not labeld road
            }
        } else {
            std::string sp_str = maxspeed.substr(0,2);
            sl = atof(sp_str.c_str());
            w = distance / sl;
        }

        edge_dict[from].push_back(std::make_pair(to, w));
        if (oneway != "yes") {
            edge_dict[to].push_back(std::make_pair(from,w));
        }
    }
}

double* TravelTool::QueryByCSV(const char *file_path) {
    double eps = 0; // error bound
    ANNidxArray nnIdx = new ANNidx[1];
    ANNdistArray dists = new ANNdist[1];

    std::vector<int> query_nodes;
    boost::unordered_map<int, std::vector<std::pair<std::string, double> > > sourceDict;

    io::CSVReader<3> query_in(file_path);
    query_in.read_header(io::ignore_extra_column, "BLOCKID10", "long", "lat");
    std::string blockid; double lng; double lat;
    int q_id;
    while(query_in.read_row(blockid, lng, lat)){
        double* pt = new double[2];
        pt[0] = lng;
        pt[1] = lat;
        kd_tree->annkSearch(pt, 1, nnIdx, dists, eps);
        q_id = nnIdx[0];
        if (sourceDict.find( q_id ) == sourceDict.end()) {
            query_nodes.push_back(q_id);
        }
        sourceDict[q_id].push_back( std::make_pair(blockid, dists[0]/20.0) );
    }
    delete[] nnIdx;
    delete[] dists;
    delete kd_tree;

    //query_nodes.resize(100);
    int query_size = query_nodes.size();

    int *results = (int*) malloc(sizeof(int) * query_size * graph.vertexCount);
    int *sourceVertArray = (int*) malloc(sizeof(int) * query_size);
    for (size_t i=0; i<query_size; i++) sourceVertArray[i] = query_nodes[i];

    cl_int errNum;
    cl_platform_id platform;
    cl_context gpuContext;
    cl_context cpuContext;
    cl_uint numPlatforms;

    // First, select an OpenCL platform to run on.  For this example, we
    // simply choose the first available platform.  Normally, you would
    // query for all available platforms and select the most appropriate one.

    errNum = clGetPlatformIDs(1, &platform, &numPlatforms);
    printf("Number of OpenCL Platforms: %d\n", numPlatforms);
    if (errNum != CL_SUCCESS || numPlatforms <= 0) {
        printf("Failed to find any OpenCL platforms.\n");
        return 0;
    }

    // create the OpenCL context on available GPU devices
    gpuContext = clCreateContextFromType(0, CL_DEVICE_TYPE_GPU, NULL, NULL, &errNum);
    if (errNum != CL_SUCCESS) {
        printf("No GPU devices found.\n");
    }

    // Create an OpenCL context on available CPU devices
    cpuContext = clCreateContextFromType(0, CL_DEVICE_TYPE_CPU, NULL, NULL, &errNum);
    if (errNum != CL_SUCCESS) {
        printf("No CPU devices found.\n");
    }

    pt::ptime startTimeGPUCPU = pt::microsec_clock::local_time();
    /*
    // CPU
    {
        size_t szParmDataBytes;
        cl_device_id *cdDevices;
        // get the list of GPU devices associated with context
        clGetContextInfo(cpuContext, CL_CONTEXT_DEVICES, 0, NULL, &szParmDataBytes);
        cdDevices = (cl_device_id *) malloc(szParmDataBytes);
        runDijkstra(cpuContext, cdDevices[0], &graph, sourceVertArray, results, query_size);
    }
     */
    //runDijkstraMultiGPU(gpuContext, &graph, sourceVertArray, results, query_size);
    runDijkstraMultiGPUandCPU(gpuContext, cpuContext, &graph, sourceVertArray, results, query_size);

    pt::time_duration timeGPUCPU = pt::microsec_clock::local_time() - startTimeGPUCPU;
    printf("\nrunDijkstra - Multi GPU and CPU Time: %f s\n", (float)timeGPUCPU.total_milliseconds() / 1000.0f);

    clReleaseContext(gpuContext);
    clReleaseContext(cpuContext);

    free(sourceVertArray);
    free(results);

    return 0;
}