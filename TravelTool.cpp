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

TravelTool::~TravelTool()
{
    free(vertex_array);
    free(edge_array);
    free(weight_array);

    if (kd_tree) delete kd_tree;
}

void TravelTool::InitCPUGPU()
{
}

void TravelTool::GetTravelRoute(double from_lat, double from_lon, double to_lat, double to_lon) {

}

void TravelTool::ComputeDistanceMatrix()
{
}

void TravelTool::BuildKdTree(int node_cnt, double** xy)
{
    if (kd_tree != 0) {
        delete kd_tree;
    }
    kd_tree = new ANNkd_tree(xy, node_cnt, 2);
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

double* TravelTool::Query(std::vector<double> lats, std::vector<double> lons) {
    return 0;
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
        has_gpus = true;
        printf("No GPU devices found.\n");
    }

    // Create an OpenCL context on available CPU devices
    cpuContext = clCreateContextFromType(0, CL_DEVICE_TYPE_CPU, NULL, NULL, &errNum);
    if (errNum != CL_SUCCESS) {
        has_cpu = true;
        printf("No CPU devices found.\n");
    }

    pt::ptime startTimeGPUCPU = pt::microsec_clock::local_time();
    //if (has_gpus && has_cpu) {
    runDijkstraMultiGPU(gpuContext, &graph, sourceVertArray,
                                  results, query_size);
    //}
    pt::time_duration timeGPUCPU = pt::microsec_clock::local_time() - startTimeGPUCPU;
    printf("\nrunDijkstra - Multi GPU and CPU Time: %f s\n", (float)timeGPUCPU.total_milliseconds() / 1000.0f);

    clReleaseContext(gpuContext);
    clReleaseContext(cpuContext);

    free(sourceVertArray);
    free(results);

    return 0;
}