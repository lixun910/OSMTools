//
// Created by Xun Li on 12/20/18.
//
#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>

#include "ogrsf_frmts.h"

#include "Downloader.h"
#include "Roads.h"

#ifndef PI
#define PI 3.141592653589793238463
#endif

#define PI_OVER_180 0.017453292519943

using namespace OSMTools;

// trim from start (in place)
static inline void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
        return !std::isspace(ch);
    }));
}

// trim from end (in place)
static inline void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

// trim from both ends (in place)
static inline void trim(std::string &s) {
    ltrim(s);
    rtrim(s);
}

// trim from start (copying)
static inline std::string ltrim_copy(std::string s) {
    ltrim(s);
    return s;
}

// trim from end (copying)
static inline std::string rtrim_copy(std::string s) {
    rtrim(s);
    return s;
}

// trim from both ends (copying)
static inline std::string trim_copy(std::string s) {
    trim(s);
    return s;
}

Roads::Roads()
{
    // max_query_area_size=50*1000*50*1000
    base_url = "http://www.overpass-api.de/api/interpreter";
}

Roads::~Roads() {}

std::string Roads::GetOSMFilter(RoadType road_type)
{
    if (road_type == OSMTools::drive) {
        return "way[\"highway\"][\"highway\"!~\"cycleway|footway|path|pedestrian|steps|track|proposed|construction|bridleway|abandoned|platform|raceway|service\"][\"motor_vehicle\"!~\"no\"][\"motorcar\"!~\"no\"][\"service\"!~\"parking|parking_aisle|driveway|emergency_access\"]";
    } else if (road_type == OSMTools::walk) {
        return "way[\"highway\"][\"highway\"!~\"motor|proposed|construction|abandoned|platform|raceway\"][\"foot\"!~\"no\"][\"pedestrians\"!~\"no\"]";
    }
    return "";
}

double Roads::ComputeArcDist(double lat1, double lon1, double lat2, double lon2)
{
    // degree to rad
    lat1 = lat1 * PI_OVER_180;
    lon1 = lon1 * PI_OVER_180;
    lat2 = lat2 * PI_OVER_180;
    lon2 = lon2 * PI_OVER_180;

    // this is the haversine formula which is particularly well-conditioned
    double d_lat_ovr_2 = (lat2-lat1)/2.0;
    double sin_sq_d_lat_ovr_2 = sin(d_lat_ovr_2);
    sin_sq_d_lat_ovr_2 *= sin_sq_d_lat_ovr_2;
    double d_lon_ovr_2 = (lon2-lon1)/2.0;
    double sin_sq_d_lon_ovr_2 = sin(d_lon_ovr_2);
    sin_sq_d_lon_ovr_2 *= sin_sq_d_lon_ovr_2;

    double a = sin_sq_d_lat_ovr_2 + cos(lat1)*cos(lat2) * sin_sq_d_lon_ovr_2;
    return 2.0* atan2(sqrt(a),sqrt(1.0-a));
}

bool Roads::DownloadByBoundingBox(double lat_min, double lng_min,
        double lat_max, double lng_max,
        OSMTools::RoadType road_type,  const char *file_name)
{
    bool flag = false;

    std::string osm_filter = GetOSMFilter(road_type);
    if (osm_filter.empty()) return false;

    std::ostringstream ss;
    ss << base_url << "?";
    ss << "data=[out:json][timeout:180];area[name=\"Chicago\"];";
    ss << osm_filter;
    //ss << "(" << lat_min << "," << lng_min << ",";
    //ss << lat_max << "," << lng_max << ")";
    ss << ";out;";

    std::string url = ss.str();

    Downloader down;
    flag = down.Get(url.c_str(), 0, file_name);

    return flag;
}

std::string Roads::GetValueFromLine(std::string line, bool quoted)
{
    // example:  "id": 237667011,
    // example:  "name": "King Drive & 25th Street",
    if (line.empty()) return "";

    size_t start = line.find(":");
    if (start == std::string::npos) return "";

    size_t end = line.length() - 1;
    if (line.find(",") != std::string::npos) {
        end = end - 1; // ending with comma
    }
    std::string tmp = line.substr(start + 1, end - start);

    if (quoted) {
        start = tmp.find("\"");
        if (start != std::string::npos) {
            end = tmp.length() - 2; // right-quote
            tmp = tmp.substr(start + 1, end - start);
        }
    }
    return tmp;
}

void Roads::ReadOSMNodes(const char *file_name)
{
    if (file_name == 0) return;

    id_map.clear();
    lat_arr.clear();
    lon_arr.clear();
    speed_arr.clear();

    int state = 0, cnt = 0, edge_cnt = 0, start = 0, end = 0;
    std::ifstream infile(file_name);
    std::string line, tmp, tmp_edge;
    std::vector<std::string> tmp_edge_nodes;
    std::vector<std::string> tmp_edge_property;

    while (std::getline(infile, line)) {
        if (line.find("node") != std::string::npos &&
            line.find("type") != std::string::npos) {
            state = 1;
            continue;
        } else if (line.find("way") != std::string::npos &&
                   line.find("type") != std::string::npos) {
            state = 5;
            continue;
        }

        if (state == 1) {
            // when node is found previously
            if (line.find("id") != std::string::npos) {
                tmp = GetValueFromLine(line, false);
                if (!tmp.empty()) {
                    trim(tmp);
                    id_map[tmp] = cnt++;

                    state = 2;
                }
            }
        } else if (state == 2) {
            // when id is found
            if (line.find("lat") != std::string::npos) {
                tmp = GetValueFromLine(line, false);
                if (!tmp.empty()) {
                    lat_arr.push_back(std::stod(tmp));
                    state = 3;
                }
            }
        } else if (state == 3) {
            // when lat is found
            if (line.find("lon") != std::string::npos) {
                tmp = GetValueFromLine(line, false);
                if (!tmp.empty()) {
                    lon_arr.push_back(std::stod(tmp));
                    state = 4;
                }
            }
        } else if (state == 4) {
            if (line.find("maxspeed") != std::string::npos) {
                tmp = GetValueFromLine(line);
                speed_arr.push_back(tmp);
                state = 0;
            }
        } else if (state == 5) {
            // way id
            if (line.find("id") != std::string::npos) {
                tmp_edge = GetValueFromLine(line);
                if (!tmp_edge.empty()) {
                    edge_arr.push_back(tmp_edge);
                    if (tmp_edge_property.empty() == false) {
                        edge_properties.push_back(tmp_edge_property);
                    }
                    tmp_edge_nodes.clear();
                    tmp_edge_property.clear();
                    tmp_edge_property.resize(5);
                    state = 6;
                }
            }
        } else if (state == 6) {
            if (line.find("nodes") != std::string::npos) {
                // start parsing nodes
                state = 7;
                continue;
            }
        } else if (state == 7) {
            if (line.find("]") != std::string::npos) {
                // end parsing nodes
                edges.push_back(tmp_edge_nodes);
                state = 8;
                continue;
            } else if (line.empty()==false){
                if (line.find(",") != std::string::npos) {
                    tmp = line.substr(0, line.length()-1); // ending comma
                }
                trim(tmp);
                tmp_edge_nodes.push_back(tmp);
            }
        } else if (state == 8) {
            if (line.find("highway") != std::string::npos) {
                tmp = GetValueFromLine(line);
                tmp_edge_property[0] = tmp;
            } else if (line.find("name") != std::string::npos) {
                tmp = GetValueFromLine(line);
                tmp_edge_property[1] = tmp;
            } else if (line.find("county") != std::string::npos) {
                tmp = GetValueFromLine(line);
                tmp_edge_property[2] = tmp;
            } else if (line.find("oneway") != std::string::npos) {
                tmp = GetValueFromLine(line);
                tmp_edge_property[3] = tmp;
            } else if (line.find("maxspeed") != std::string::npos) {
                tmp = GetValueFromLine(line);
                tmp_edge_property[4] = tmp;
            }
        }
    }
    // record last edge property
    edge_properties.push_back(tmp_edge_property);

    printf( "%d == %d\n", edges.size(), edge_properties.size());
}

void Roads::SaveOSMToShapefile(const char *file_name)
{
    std::string shp_file_name = file_name;
    shp_file_name = shp_file_name.substr(0, shp_file_name.rfind("."));
    shp_file_name = shp_file_name + ".shp";

    const char *pszDriverName = "ESRI Shapefile";
    GDALDriver *poDriver;
    GDALAllRegister();
    poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName );
    if( poDriver == NULL ) {
        printf( "%s driver not available.\n", pszDriverName );
        return;
    }
    GDALDataset *poDS;
    poDS = poDriver->Create(shp_file_name.c_str(), 0, 0, 0, GDT_Unknown, NULL );
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
    OGRFieldDefn oField("Length", OFTReal);
    //oField.SetWidth(32);
    if( poLayer->CreateField( &oField ) != OGRERR_NONE ) {
        printf( "Creating Name field failed.\n" );
        GDALClose( poDS );
        return;
    }
    for (size_t i=0; i<edges.size(); ++i) {
        OGRFeature *poFeature;
        poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
        for (size_t c=0; c<5; ++c) {
            if (c == 0) {
                poFeature->SetField(prop_names[c], edge_arr[i].c_str());
            } else {
                const char *val = edge_properties[i][c-1].c_str();
                poFeature->SetField(prop_names[c], val);
            }
        }
        OGRLineString line;
        size_t n_nodes = edges[i].size();
        line.setNumPoints(n_nodes);
        for (size_t node_i=0; node_i < n_nodes; ++node_i) {
            std::string node_id = edges[i][node_i];
            int node_idx = id_map[node_id];
            if (node_idx >= 0 ) {
                OGRPoint pt;
                pt.setX(lon_arr[node_idx]);
                pt.setY(lat_arr[node_idx]);
                line.setPoint(node_i, &pt);
            }
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