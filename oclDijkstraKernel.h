//
// Book:      OpenCL(R) Programming Guide
// Authors:   Aaftab Munshi, Benedict Gaster, Timothy Mattson, James Fung, Dan Ginsburg
// ISBN-10:   0-321-74964-2
// ISBN-13:   978-0-321-74964-2
// Publisher: Addison-Wesley Professional
// URLs:      http://safari.informit.com/9780132488006/
//            http://www.openclprogrammingguide.com
//

//
//
//  Description:
//      Implementation of Dijkstra's Single-Source Shortest Path (SSSP) algorithm on the GPU.
//      The basis of this implementation is the paper:
//
//          "Accelerating large graph algorithms on the GPU using CUDA" by
//          Parwan Harish and P.J. Narayanan
//
//
//  Author:
//      Dan Ginsburg
//      <daniel.ginsburg@childrens.harvard.edu>
//
//  Children's Hospital Boston
//
#ifndef DIJKSTRA_KERNEL_H
#define DIJKSTRA_KERNEL_H


#ifdef __APPLE__
    #include <OpenCL/cl.h>
#else
    #include <CL/cl.h>
#endif

static char cl_dir[1024];

//  Function prototypes
//
bool maskArrayEmpty(int *maskArray, int count);

///
//  Types
//


///
//  Utility functions adapted from NVIDIA GPU Computing SDK
//
void checkErrorFileLine(int errNum, int expected, const char* file, const int lineNumber);
cl_device_id getDev(cl_context cxGPUContext, unsigned int nr);
cl_device_id getFirstDev(cl_context cxGPUContext);
void checkErrorFileLine(int errNum, int expected, const char* file, const int lineNumber);
int roundWorkSizeUp(int groupSize, int globalSize);

///
//  Types
//
//
//  This data structure and algorithm implementation is based on
//  Accelerating large graph algorithms on the GPU using CUDA by
//  Parwan Harish and P.J. Narayanan
//
typedef struct
{
    // (V) This contains a pointer to the edge list for each vertex
    int *vertexArray;

    // Vertex count
    int vertexCount;

    // (E) This contains pointers to the vertices that each edge is attached to
    int *edgeArray;

    // Edge count
    int edgeCount;

    // (W) Weight array
    int *weightArray;

} GraphData;

// This structure is used in the multi-GPU implementation of the algorithm.
// This structure defines the workload for each GPU.  The code chunks up
// the work on a per-GPU basis.
typedef struct
{
    // Context
    cl_context context;
    
    // Device number to run algorithm on
    cl_device_id deviceId;
    
    // Pointer to graph data
    GraphData *graph;
    
    // Source vertex indices to process
    int *sourceVertices;
    
    // End vertex indices to process
    int *endVertices;
    
    // Results of processing
    int *outResultCosts;
    
    // Number of results
    int numResults;
    
} DevicePlan;



///
/// Run Dijkstra's shortest path on the GraphData provided to this function.  This
/// function will compute the shortest path distance from sourceVertices[n] ->
/// endVertices[n] and store the cost in outResultCosts[n].  The number of results
/// it will compute is given by numResults.
///
/// This version of the function will run the algorithm on either just the CPU,
/// CPU + GPU, GPU, or Multi GPU depending on what compute resources are available
/// on the system.
///
/// \param graph Structure containing the vertex, edge, and weight arra
///              for the input graph
/// \param startVertices Indices into the vertex array from which to
///                      start the search
/// \param outResultsCosts A pre-allocated array where the results for
///                        each shortest path search will be written.
///                        This must be sized numResults * graph->numVertices.
/// \param numResults Should be the size of all three passed inarrays
///
void runDijkstraOpenCL( GraphData* graph, int *sourceVertices,
                        int *outResultCosts, int numResults );

///
/// Run Dijkstra's shortest path on the GraphData provided to this function.  This
/// function will compute the shortest path distance from sourceVertices[n] ->
/// endVertices[n] and store the cost in outResultCosts[n].  The number of results
/// it will compute is given by numResults.
///
/// This function will run the algorithm on a single GPU.
///
/// \param gpuContext Current context, must be created by caller
/// \param deviceId The device ID on which to run the kernel.  This can
///                 be determined externally by the caller or the multi
///                 GPU version will automatically split the work across
///                 devices
/// \param graph Structure containing the vertex, edge, and weight arra
///              for the input graph
/// \param startVertices Indices into the vertex array from which to
///                      start the search
/// \param outResultsCosts A pre-allocated array where the results for
///                        each shortest path search will be written.
///                        This must be sized numResults * graph->numVertices.
/// \param numResults Should be the size of all three passed inarrays
///
void runDijkstra( cl_context context, cl_device_id deviceId, GraphData* graph,
                  int *sourceVertices, int *outResultCosts, int numResults );


///
/// Run Dijkstra's shortest path on the GraphData provided to this function.  This
/// function will compute the shortest path distance from sourceVertices[n] ->
/// endVertices[n] and store the cost in outResultCosts[n].  The number of results
/// it will compute is given by numResults.
///
/// This function will run the algorithm on as many GPUs as is available.  It will
/// create N threads, one for each GPU, and chunk the workload up to perform
/// (numResults / N) searches per GPU.
///
/// \param gpuContext Current GPU context, must be created by caller
/// \param graph Structure containing the vertex, edge, and weight arra
///              for the input graph
/// \param startVertices Indices into the vertex array from which to
///                      start the search
/// \param outResultsCosts A pre-allocated array where the results for
///                        each shortest path search will be written.
///                        This must be sized numResults * graph->numVertices.
/// \param numResults Should be the size of all three passed inarrays
///
///
void runDijkstraMultiGPU( cl_context gpuContext, GraphData* graph, int *sourceVertices,
                          int *outResultCosts, int numResults );

///
/// Run Dijkstra's shortest path on the GraphData provided to this function.  This
/// function will compute the shortest path distance from sourceVertices[n] ->
/// endVertices[n] and store the cost in outResultCosts[n].  The number of results
/// it will compute is given by numResults.
///
/// This function will run the algorithm on as many GPUs as is available along with
/// the CPU.  It will create N threads, one for each device, and chunk the workload up to perform
/// (numResults / N) searches per device.
///
/// \param gpuContext Current GPU context, must be created by caller
/// \param cpuContext Current CPU context, must be created by caller
/// \param graph Structure containing the vertex, edge, and weight arra
///              for the input graph
/// \param startVertices Indices into the vertex array from which to
///                      start the search
/// \param outResultsCosts A pre-allocated array where the results for
///                        each shortest path search will be written.
///                        This must be sized numResults * graph->numVertices.
/// \param numResults Should be the size of all three passed inarrays
///
///
void runDijkstraMultiGPUandCPU( const char* cl_dir, cl_context gpuContext, cl_context cpuContext, GraphData* graph,
                                int *sourceVertices, int *outResultCosts, int numResults );


///
/// Run Dijkstra's shortest path on the GraphData provided to this function.  This
/// function will compute the shortest path distance from sourceVertices[n] ->
/// endVertices[n] and store the cost in outResultCosts[n].  The number of results
/// it will compute is given by numResults.
///
/// This is a CPU *REFERENCE* implementation for use as a fallback.
///
/// \param graph Structure containing the vertex, edge, and weight arra
///              for the input graph
/// \param startVertices Indices into the vertex array from which to
///                      start the search
/// \param outResultsCosts A pre-allocated array where the results for
///                        each shortest path search will be written.
///                        This must be sized numResults * graph->numVertices.
/// \param numResults Should be the size of all three passed inarrays
///
void runDijkstraMT ( GraphData* graph, int *sourceVertices,
                     int *outResultCosts, int numResults );

void runDijkstraRef( GraphData* graph, int *sourceVertices,
                     int *outResultCosts, int start, int end);

void runSSSPGPU(const char* cl_dir, cl_context gpuContext, cl_context cpuContext, GraphData* graph,
                int *sourceVertices, int *outResultCosts, int numResults );

void runSSSP( const char* dir, cl_context context, cl_device_id deviceId, GraphData* graph,
             int *sourceVertices, int *outResultCosts, int numResults);

#endif // DIJKSTRA_KERNEL_H
