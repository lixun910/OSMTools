//
// Created by Xun Li on 12/22/18.
//
#include <boost/date_time/posix_time/posix_time.hpp>
#include <wx/stdpaths.h>
#include <wx/filename.h>
#include "TravelTool.h"


#ifndef PI
#define PI 3.141592653589793238463
#endif

#ifndef PI_OVER_180
#define PI_OVER_180 0.017453292519943
#endif

#ifndef EARTH_RADIUS
#define EARTH_RADIUS 6372795  // meters
#endif

using namespace OSMTools;
namespace pt = boost::posix_time;

TravelTool::TravelTool()
{

}

TravelTool::TravelTool(std::vector<OGRFeature*> in_roads,
        std::vector<OGRFeature*> in_query_points)
{
    speed_limit_dict["road"]=32;
    speed_limit_dict["motorway"] = 96;
    speed_limit_dict["motorway_link"] = 48;
    speed_limit_dict["motorway_junction"] = 48;
    speed_limit_dict["trunk"] = 80;
    speed_limit_dict["trunk_link"] = 40;
    speed_limit_dict["primary"] = 48;
    speed_limit_dict["primary_link"] = 32;
    speed_limit_dict["secondary"] = 40;
    speed_limit_dict["secondary_link"] = 40;
    speed_limit_dict["tertiary"] = 32;
    speed_limit_dict["tertiary_link"] = 32;
    speed_limit_dict["residential"] = 24;
    speed_limit_dict["living_street"] = 16;
    speed_limit_dict["service"] = 16;
    speed_limit_dict["track"] = 32;
    speed_limit_dict["pedestrian"] = 3.2;
    speed_limit_dict["services"] = 3.2;
    speed_limit_dict["bus_guideway"] = 3.2;
    speed_limit_dict["path"] = 8;
    speed_limit_dict["cycleway"] = 16;
    speed_limit_dict["footway"] = 3.2;
    speed_limit_dict["bridleway"] = 3.2;
    speed_limit_dict["byway"] = 3.2;
    speed_limit_dict["steps"] = 0.16;
    speed_limit_dict["unclassified"] = 24;
    speed_limit_dict["lane"] = 16;
    speed_limit_dict["opposite_lane"] = 16;
    speed_limit_dict["opposite"] = 16;
    speed_limit_dict["grade1"] = 16;
    speed_limit_dict["grade2"] = 16;
    speed_limit_dict["grade3"] = 16;
    speed_limit_dict["grade4"] = 16;
    speed_limit_dict["grade5"] = 16;
    speed_limit_dict["roundabout"] = 40;

    roads = in_roads;
    query_points = in_query_points;

    PreprocessRoads();
}

TravelTool::~TravelTool()
{
    free(vertex_array);
    free(edge_array);
    free(weight_array);
}

wxString TravelTool::GetExeDir()
{
    wxString exePath = wxStandardPaths::Get().GetExecutablePath();
    wxFileName exeFile(exePath);
    wxString exeDir = exeFile.GetPathWithSep();
    return exeDir;

}
void TravelTool::PreprocessRoads()
{
    pt::ptime startTimeGPUCPU = pt::microsec_clock::local_time();
    
    if (roads.empty() == true) return;

    size_t n_roads = roads.size();
    OGRFeature* feature;
    OGRGeometry* geom;
    OGRLineString* line;
    int i, j, idx, n_pts, node_count=0;

    // read ways
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

                if (nodes_dict.find(rd_pt) == nodes_dict.end()) {
                    nodes_dict[rd_pt] = node_count;
                    node_appearance[node_count] = 1;
                    nodes.push_back(pt);
                    node_count ++;
                } else {
                    idx = nodes_dict[rd_pt];
                    node_appearance[idx] += 1;
                }
                // end points: [wayid, wayid..]
                if (j==0 || j == n_pts-1) {
                    idx = nodes_dict[rd_pt];
                    endpoint_dict[idx].push_back(i);
                }
                e.push_back(pt); // todo: possible memory issue
            }
        }
        edges.push_back(e);
    }

    node_count = node_appearance.size();
    nodes_flag.resize(node_count, false);

    // create a kdtree using nodes in ways
    double** xy = new double*[node_count];
    for (i=0; i<node_count; ++i) {
        OGRPoint pt = nodes[i];
        xy[i] = new double[2];
        xy[i][0] = pt.getX();
        xy[i][1] = pt.getY();
    }
    kd_tree = new ANNkd_tree(xy, node_count, 2);

    // using query points to find anchor points from roads
    double eps = 0.00000001; // error bound
    ANN_DIST_TYPE = 2;

    int anchor_cnt = 0, query_size = query_points.size();
    for (i=0; i<query_size; ++i) {
        feature = query_points[i];
        geom = feature->GetGeometryRef();
        OGRPoint* m_pt = (OGRPoint*)geom;
        double* q_pt = new double[2];
        q_pt[0] = m_pt->getX();
        q_pt[1] = m_pt->getY();
        ANNidxArray nnIdx = new ANNidx[2];
        ANNdistArray dists = new ANNdist[2];
        kd_tree->annkSearch(q_pt, 2, nnIdx, dists, eps);
        int q_id = nnIdx[0];
        anchor_points[q_id] = true;
        if (source_dict.find(q_id) == source_dict.end()) {
            query_nodes.push_back(q_id);
            anchor_cnt++;
        }
        source_dict[q_id].push_back(
                std::make_pair(i, dists[0]/20.0) );
        delete[] q_pt;
        delete[] nnIdx;
        delete[] dists;
    }

    delete kd_tree;
    for (i=0; i<node_count; ++i) delete[] xy[i];
    delete[] xy;

    // build graph
    // remove circle way
    int from, to;
    std::string highway, oneway, maxspeed;
    double speed_limit = 20.0, length, cost;
    oneway_dict.resize(edges.size(), false);
    for (i=0; i<edges.size(); ++i) {
        n_pts = edges[i].size();
        OGRPoint start = edges[i][0];
        OGRPoint end = edges[i][n_pts-1];
        if (start.Equals(&end)) {
            removed_edges[i] = true;
        }
        // compute pair_cost
        feature = roads[i];
        highway = feature->GetFieldAsString("highway");
        oneway = feature->GetFieldAsString("oneway");
        maxspeed = feature->GetFieldAsString("maxspeed");
        if (maxspeed.empty()) {
            if (speed_limit_dict.find(highway) != speed_limit_dict.end()) {
                speed_limit = speed_limit_dict[highway];
            }
        } else {
            std::string sp_str = maxspeed.substr(0,2);
            speed_limit = atoi(sp_str.c_str()); // mile
            speed_limit *= 1.6; // km
        }
        oneway_dict[i] = std::strcmp(oneway.c_str(),"yes") == 0 ? true : false;

        for (j=0; j<n_pts-1; ++j) {
            start = edges[i][j];
            end = edges[i][j+1];
            RD_POINT rd_from = std::make_pair(start.getX(), start.getY());
            RD_POINT rd_to = std::make_pair(end.getX(), end.getY());
            from = nodes_dict[rd_from];
            to = nodes_dict[rd_to];
            length = ComputeArcDist(start, end);
            cost = length / speed_limit;
            pair_cost[std::make_pair(from, to)] = cost;
            if (oneway_dict[i]==false) {
                pair_cost[std::make_pair(to, from)] = cost;
            }
        }
    }

    // concat ways
    int node_idx, w1, w2;
    bool is_merged;
    std::vector<int> conn_ways;
    for (i=0; i<edges.size(); ++i) {
        if (removed_edges.find(i) != removed_edges.end()) {
            // this edge has been merged with other edge already
            // so skip
            continue;
        }
        n_pts = edges[i].size();
        OGRPoint start = edges[i][0];
        RD_POINT rd_start = std::make_pair(start.getX(), start.getY());
        node_idx = nodes_dict[rd_start];
        conn_ways = endpoint_dict[node_idx];
        is_merged = false;
        if (conn_ways.size() == 2) { // two ways share node_idx (start)
            w1 = i;
            w2 = conn_ways[0] == i ? conn_ways[1] : conn_ways[0];
            if (removed_edges.find(w2) == removed_edges.end()) {
                if (MergeTwoWaysByStart(w1, w2)) {
                    removed_edges[w2] = true;
                    is_merged = true;
                }
            }
        }

        OGRPoint end = edges[i][n_pts-1];
        if (start.Equals(&end)) continue; // cont if two ends are same

        RD_POINT rd_end = std::make_pair(end.getX(), end.getY());
        node_idx = nodes_dict[rd_end];
        conn_ways = endpoint_dict[node_idx];
        if (conn_ways.size() == 2) { // tow ways share node_idx (end)
            w1 = i;
            w2 = conn_ways[0] == i ? conn_ways[1] : conn_ways[0];
            if (removed_edges.find(w2) == removed_edges.end()) {
                if (MergeTwoWaysByEnd(w1, w2)) {
                    removed_edges[w2] = true;
                    is_merged = true;
                }
            }
        }
        if (is_merged) {
            i--;
            // reprocessing current way since it's merged w/ other ways
            // and therefore its content has been updated
        }
    }
    //SaveMergedRoads("/Users/xun/Desktop/merge.shp");

    // simplify ways
    for (i=0; i<edges.size(); ++i) {
        if (removed_edges.find(i) != removed_edges.end()) {
            continue;
        }
        n_pts = edges[i].size();
        if (n_pts < 3) continue; // if way has 2 or 1 nodes, just skip

        double total_cost = 0;
        OGRPoint pt, prev_pt = edges[i][n_pts-1];
        std::pair<int, int> from_to;

        for (j=n_pts-2; j>=0; --j) {
            pt = edges[i][j];
            RD_POINT rd_pt = std::make_pair(pt.getX(), pt.getY());
            node_idx = nodes_dict[rd_pt];
            from_to.first = node_idx;
            from_to.second = nodes_dict[std::make_pair(prev_pt.getX(), prev_pt.getY())];

            if (j> 0 && node_appearance[node_idx] == 1 &&
                pair_cost.find(from_to) != pair_cost.end() &&
                anchor_points.find(node_idx) == anchor_points.end())
            {
                total_cost += pair_cost[from_to];
                edges[i].erase(edges[i].begin() + j);
            } else {
                total_cost += pair_cost[from_to];
                // add {from, to, dist} to edge_dict
                AddEdge(i, edges[i][j], edges[i][j+1], total_cost);
                total_cost = 0; // reset total_cost to 0
            }
            prev_pt = pt;
        }
    }

    //SaveMergedRoads("/Users/xun/Desktop/simplified.shp");

    // re-index nodes and edges
    int valid_index = 0;
    boost::unordered_map<int, int> index_map;
    boost::unordered_map<int, int> rev_index_map;
    for (i=0; i<node_count; ++i) {
        if (nodes_flag[i] == true) {
            index_map[valid_index] = i;
            rev_index_map[i] = valid_index;
            valid_index ++;
        }
    }

    num_nodes = valid_index;
    num_edges = 0;
    boost::unordered_map<int, std::vector<std::pair<int, double> > >::iterator it;
    for (it = edges_dict.begin(); it!= edges_dict.end(); ++it) {
        num_edges += it->second.size();
    }

    vertex_array = (int*) malloc(num_nodes * sizeof(int));
    edge_array = (int*)malloc(num_edges * sizeof(int));
    weight_array = (float*)malloc(num_edges * sizeof(float));

    graph.vertexCount = num_nodes;
    graph.vertexArray = vertex_array;
    graph.edgeCount = num_edges;
    graph.edgeArray = edge_array;
    graph.weightArray = weight_array;

    size_t offset = 0, e_idx = 0, nbr_idx;
    for(i = 0; i < graph.vertexCount; i++) {
        // for each valid node
        graph.vertexArray[i] = offset;
        node_idx = index_map[i];
        std::vector<std::pair<int, double> > nbrs = edges_dict[node_idx];

        for (j=0; j<nbrs.size(); ++j) {
            nbr_idx = rev_index_map[ nbrs[j].first ];
            cost = nbrs[j].second;
            graph.edgeArray[e_idx] = nbr_idx;
            graph.weightArray[e_idx++] = cost;
        }

        offset += nbrs.size();
    }

    query_size = query_nodes.size();
    size_t results_mem = sizeof(int) * (size_t)query_size * (size_t)graph.vertexCount;
    float *results = (float*) malloc(results_mem);
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
        return;
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

    char cl_dir[1024];
    wxString current_dir = GetExeDir();
    strncpy(cl_dir, (const char*)current_dir.mb_str(wxConvUTF8), 1023);


    runDijkstraMultiGPUandCPU(cl_dir, gpuContext, cpuContext, &graph, sourceVertArray, results, query_size);
    //runDijkstraMT(&graph, sourceVertArray, results, query_size);
    pt::time_duration timeGPUCPU = pt::microsec_clock::local_time() - startTimeGPUCPU;
    printf("\nrunDijkstra - Multi GPU and CPU Time: %f s\n", (float)timeGPUCPU.total_milliseconds() / 1000.0f);

    clReleaseContext(gpuContext);
    clReleaseContext(cpuContext);

    free(sourceVertArray);
    free(results);
}

void TravelTool::AddEdge(int way_idx, OGRPoint& from, OGRPoint& to, double cost)
{
    RD_POINT rd_from = std::make_pair(from.getX(), from.getY());
    RD_POINT rd_to = std::make_pair(to.getX(), to.getY());
    int from_idx = nodes_dict[rd_from];
    int to_idx = nodes_dict[rd_to];

    nodes_flag[from_idx] = true;
    nodes_flag[to_idx] = true;

    edges_dict[from_idx].push_back(std::make_pair(to_idx, cost));
    if (oneway_dict[way_idx] == false) {
        edges_dict[to_idx].push_back(std::make_pair(from_idx, cost));
    }
}

bool TravelTool::MergeTwoWaysByStart(int w1, int w2)
{
    std::vector<OGRPoint>& e1 = edges[w1];
    std::vector<OGRPoint>& e2 = edges[w2];
    OGRPoint start1 = e1[0];
    OGRPoint start2 = e2[0];
    OGRPoint end2 = e2[e2.size() -1];

    if (start1.Equals(&start2)) {
        // reverse w2, concat w1
        if (oneway_dict[w2]) return false;
        for (int i=1; i < e2.size(); ++i) {
            e1.insert(e1.begin(), e2[i]);
        }
    } else if (start1.Equals(&end2)) {
        // w2 concat w1
        for (int i=e2.size()-2; i >= 0; --i) {
            e1.insert(e1.begin(), e2[i]);
        }
    } else {
        return false;
    }
    RD_POINT rd_pt = std::make_pair(start1.getX(), start1.getY());
    int node_idx = nodes_dict[rd_pt];
    node_appearance[node_idx] -= 1;
    return true;
}

bool TravelTool::MergeTwoWaysByEnd(int w1, int w2)
{
    std::vector<OGRPoint>& e1 = edges[w1];
    std::vector<OGRPoint>& e2 = edges[w2];
    OGRPoint end1 = e1[e1.size()-1];
    OGRPoint start2 = e2[0];
    OGRPoint end2 = e2[e2.size() -1];

    if (end1.Equals(&start2)) {
        // w1, concat w2
        for (int i=1; i < e2.size(); ++i) {
            e1.push_back(e2[i]);
        }
    } else if (end1.Equals(&end2)) {
        // w1 concat reversed w2
        if (oneway_dict[w2]) return false;
        for (int i=e2.size()-2; i >= 0; --i) {
            e1.push_back(e2[i]);
        }
    } else {
        return false;
    }
    RD_POINT rd_pt = std::make_pair(end1.getX(), end1.getY());
    int node_idx = nodes_dict[rd_pt];
    node_appearance[node_idx] -= 1;
    return true;
}

double TravelTool::ComputeArcDist(OGRPoint& from, OGRPoint& to)
{
    double lat1 = from.getX();
    double lon1 = from.getY();
    double lat2 = to.getX();
    double lon2 = to.getY();
    // degree to rad
    lat1 = lat1 * PI_OVER_180;
    lon1 = lon1 * PI_OVER_180;
    lat2 = lat2 * PI_OVER_180;
    lon2 = lon2 * PI_OVER_180;

    // this is the haversine formula which is particularly well-conditioned
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double a = pow(sin(dlat/2.0), 2);
    double b = cos(lat1) * cos(lat2) * pow(sin(dlon/2.0),2);
    return 2 * EARTH_RADIUS * asin(sqrt(a+b));
}

void TravelTool::SaveMergedRoads(const char* shp_file_name)
{
    const char *pszDriverName = "ESRI Shapefile";
    GDALDriver *poDriver;
    GDALAllRegister();
    poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName );
    if( poDriver == NULL ) {
        printf( "%s driver not available.\n", pszDriverName );
        return;
    }
    GDALDataset *poDS;
    poDS = poDriver->Create(shp_file_name, 0, 0, 0, GDT_Unknown, NULL );
    if( poDS == NULL ) {
        printf( "Creation of output file failed.\n" );
        return;
    }
    OGRLayer *poLayer;
    poLayer = poDS->CreateLayer("roads", NULL, wkbLineString, NULL );
    if( poLayer == NULL ) {
        printf( "Layer creation failed.\n" );
        GDALClose( poDS );
        return;
    }
    // wayid, highway, name, county, oneway, maxspeed, length,
    const char* prop_names[255] = {"wayid", "highway", "name", "county", "oneway", "maxspeed"};
    for (size_t i=0; i<6; ++i) {
        OGRFieldDefn oField(prop_names[i], OFTString );
        oField.SetWidth(32);
        if( poLayer->CreateField( &oField ) != OGRERR_NONE ) {
            printf( "Creating Name field failed.\n" );
            GDALClose( poDS );
            return;
        }
    }
    for (size_t i=0; i<edges.size(); ++i) {
        if (removed_edges.find(i) != removed_edges.end()) {
            continue;
        }
        OGRFeature *poFeature;
        poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
        for (size_t c=0; c<6; ++c) {
            poFeature->SetField(c, roads[i]->GetFieldAsString(c));
        }
        OGRLineString line;
        for (size_t j=0; j<edges[i].size(); ++j) {
            OGRPoint pt = edges[i][j];
            line.addPoint(&pt);
        }
        poFeature->SetGeometry(&line);
        if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE ) {
            printf( "Failed to create feature in shapefile.\n" );
            GDALClose( poDS );
            return;
        }
        OGRFeature::DestroyFeature( poFeature );
    }
    GDALClose( poDS );
}
