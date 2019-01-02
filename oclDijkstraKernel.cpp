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
#include <float.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
//#include <pthread.h>
#include <boost/thread.hpp>
#include <sstream>
#include <iostream>
#include <fstream>
#include "oclDijkstraKernel.h"

#define MAX_SOURCE_SIZE (0x100000)

///
//  Macros
//
#define checkError(a, b) checkErrorFileLine(a, b, __FILE__ , __LINE__)

///
//  Macro Options
//
#define NUM_ASYNCHRONOUS_ITERATIONS 10  // Number of async loop iterations before attempting to read results back

///



///
//  Namespaces
//
using namespace std;


///
//  Globals
//
//pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;
boost::mutex mtx;

///////////////////////////////////////////////////////////////////////////////
//
//  Private Functions
//
//

///
/// Load and build an OpenCL program from source file
/// \param gpuContext GPU context on which to load and build the program
/// \param fileName File name of source file that holds the kernels
/// \return Handle to the program
///
cl_program loadAndBuildProgram( cl_context gpuContext, const char *fileName )
{
    //pthread_mutex_lock(&mutex1);
    mtx.lock();

    cl_int errNum;
    cl_program program;

    // Load the OpenCL source code from the .cl file
    char file_path[1024];
    strncpy(file_path, cl_dir, 1024);
    strcat(file_path, fileName);
    std::ifstream kernelFile(file_path, std::ios::in);
    if (!kernelFile.is_open())
    {
        std::cerr << "Failed to open file for reading: " << fileName << std::endl;
        return NULL;
    }

    std::ostringstream oss;
    oss << kernelFile.rdbuf();

    std::string srcStdStr = oss.str();
    const char *source = srcStdStr.c_str();

    checkError(source != NULL, true);

    // Create the program for all GPUs in the context
    program = clCreateProgramWithSource(gpuContext, 1, (const char **)&source, NULL, &errNum);
    checkError(errNum, CL_SUCCESS);
    // build the program for all devices on the context
    errNum = clBuildProgram(program, 0, NULL, NULL, NULL, NULL);
    if (errNum != CL_SUCCESS)
    {
        char cBuildLog[10240];
        clGetProgramBuildInfo(program, getFirstDev(gpuContext), CL_PROGRAM_BUILD_LOG,
                              sizeof(cBuildLog), cBuildLog, NULL );

        cerr << cBuildLog << endl;
        checkError(errNum, CL_SUCCESS);
    }

    //pthread_mutex_unlock(&mutex1);
    mtx.unlock();
    return program;
}

///
///  Allocate memory for input CUDA buffers and copy the data into device memory
///
void allocateOCLBuffers(cl_context gpuContext, cl_command_queue commandQueue, GraphData *graph,
                        cl_mem *vertexArrayDevice, cl_mem *edgeArrayDevice, cl_mem *weightArrayDevice,
                        cl_mem *maskArrayDevice, cl_mem *costArrayDevice, cl_mem *updatingCostArrayDevice,
                        size_t globalWorkSize)
{
    cl_int errNum;
    cl_mem hostVertexArrayBuffer;
    cl_mem hostEdgeArrayBuffer;
    cl_mem hostWeightArrayBuffer;

    // First, need to create OpenCL Host buffers that can be copied to device buffers
    hostVertexArrayBuffer = clCreateBuffer(gpuContext, CL_MEM_COPY_HOST_PTR | CL_MEM_ALLOC_HOST_PTR,
                                           sizeof(int) * graph->vertexCount, graph->vertexArray, &errNum);
    checkError(errNum, CL_SUCCESS);

    hostEdgeArrayBuffer = clCreateBuffer(gpuContext, CL_MEM_COPY_HOST_PTR | CL_MEM_ALLOC_HOST_PTR,
                                           sizeof(int) * graph->edgeCount, graph->edgeArray, &errNum);
    checkError(errNum, CL_SUCCESS);

    hostWeightArrayBuffer = clCreateBuffer(gpuContext, CL_MEM_COPY_HOST_PTR | CL_MEM_ALLOC_HOST_PTR,
                                           sizeof(int) * graph->edgeCount, graph->weightArray, &errNum);
    checkError(errNum, CL_SUCCESS);

    // Now create all of the GPU buffers
    *vertexArrayDevice = clCreateBuffer(gpuContext, CL_MEM_READ_ONLY, sizeof(int) * globalWorkSize, NULL, &errNum);
    checkError(errNum, CL_SUCCESS);
    *edgeArrayDevice = clCreateBuffer(gpuContext, CL_MEM_READ_ONLY, sizeof(int) * graph->edgeCount, NULL, &errNum);
    checkError(errNum, CL_SUCCESS);
    *weightArrayDevice = clCreateBuffer(gpuContext, CL_MEM_READ_ONLY, sizeof(int) * graph->edgeCount, NULL, &errNum);
    checkError(errNum, CL_SUCCESS);
    *maskArrayDevice = clCreateBuffer(gpuContext, CL_MEM_READ_WRITE, sizeof(int) * globalWorkSize, NULL, &errNum);
    checkError(errNum, CL_SUCCESS);
    *costArrayDevice = clCreateBuffer(gpuContext, CL_MEM_READ_WRITE, sizeof(int) * globalWorkSize, NULL, &errNum);
    checkError(errNum, CL_SUCCESS);
    *updatingCostArrayDevice = clCreateBuffer(gpuContext, CL_MEM_READ_WRITE, sizeof(int) * globalWorkSize, NULL, &errNum);
    checkError(errNum, CL_SUCCESS);

    // Now queue up the data to be copied to the device
    errNum = clEnqueueCopyBuffer(commandQueue, hostVertexArrayBuffer, *vertexArrayDevice, 0, 0,
                                 sizeof(int) * graph->vertexCount, 0, NULL, NULL);
    checkError(errNum, CL_SUCCESS);

    errNum = clEnqueueCopyBuffer(commandQueue, hostEdgeArrayBuffer, *edgeArrayDevice, 0, 0,
                                 sizeof(int) * graph->edgeCount, 0, NULL, NULL);
    checkError(errNum, CL_SUCCESS);

    errNum = clEnqueueCopyBuffer(commandQueue, hostWeightArrayBuffer, *weightArrayDevice, 0, 0,
                                 sizeof(int) * graph->edgeCount, 0, NULL, NULL);
    checkError(errNum, CL_SUCCESS);

    clReleaseMemObject(hostVertexArrayBuffer);
    clReleaseMemObject(hostEdgeArrayBuffer);
    clReleaseMemObject(hostWeightArrayBuffer);
}

///
/// Initialize OpenCL buffers for single run of Dijkstra
///
void initializeOCLBuffers(cl_command_queue commandQueue, cl_kernel initializeKernel, GraphData *graph,
                          size_t maxWorkGroupSize)
{
    cl_int errNum;
    // Set # of work items in work group and total in 1 dimensional range
    size_t localWorkSize = maxWorkGroupSize;
    size_t globalWorkSize = roundWorkSizeUp(localWorkSize, graph->vertexCount);

    errNum = clEnqueueNDRangeKernel(commandQueue, initializeKernel, 1, NULL, &globalWorkSize, &localWorkSize,
                                    0, NULL, NULL);
    checkError(errNum, CL_SUCCESS);
}

///
/// Worker thread for running the algorithm on one of the compute devices
///
void dijkstraThread(DevicePlan *plan)
{
    runDijkstra( plan->context, plan->deviceId, plan->graph, plan->sourceVertices,
                 plan->outResultCosts, plan->numResults );
}

///
/// Gets the id of the nth device from the context (from the NVIDIA SDK)
///
cl_device_id getDev(cl_context cxGPUContext, unsigned int nr)
{
    size_t szParmDataBytes;
    cl_device_id* cdDevices;

    // get the list of GPU devices associated with context
    clGetContextInfo(cxGPUContext, CL_CONTEXT_DEVICES, 0, NULL, &szParmDataBytes);

    if( szParmDataBytes / sizeof(cl_device_id) < nr )
    {
        return (cl_device_id)-1;
    }

    cdDevices = (cl_device_id*) malloc(szParmDataBytes);

    clGetContextInfo(cxGPUContext, CL_CONTEXT_DEVICES, szParmDataBytes, cdDevices, NULL);

    cl_device_id device = cdDevices[nr];
    free(cdDevices);

    return device;
}


///
/// Gets the id of the first device from the context (from the NVIDIA SDK)
///
cl_device_id getFirstDev(cl_context cxGPUContext)
{
    size_t szParmDataBytes;
    cl_device_id* cdDevices;

    // get the list of GPU devices associated with context
    clGetContextInfo(cxGPUContext, CL_CONTEXT_DEVICES, 0, NULL, &szParmDataBytes);
    cdDevices = (cl_device_id*) malloc(szParmDataBytes);

    clGetContextInfo(cxGPUContext, CL_CONTEXT_DEVICES, szParmDataBytes, cdDevices, NULL);

    cl_device_id first = cdDevices[0];
    free(cdDevices);

    return first;
}

///
/// Gets the id of device with maximal FLOPS from the context (from NVIDIA SDK)
///
cl_device_id getMaxFlopsDev(cl_context cxGPUContext)
{
    size_t szParmDataBytes;
    cl_device_id* cdDevices;

    // get the list of GPU devices associated with context
    clGetContextInfo(cxGPUContext, CL_CONTEXT_DEVICES, 0, NULL, &szParmDataBytes);
    cdDevices = (cl_device_id*) malloc(szParmDataBytes);
    size_t device_count = szParmDataBytes / sizeof(cl_device_id);

    clGetContextInfo(cxGPUContext, CL_CONTEXT_DEVICES, szParmDataBytes, cdDevices, NULL);

    cl_device_id max_flops_device = cdDevices[0];
	int max_flops = 0;

	size_t current_device = 0;

    // CL_DEVICE_MAX_COMPUTE_UNITS
    cl_uint compute_units;
    clGetDeviceInfo(cdDevices[current_device], CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(compute_units), &compute_units, NULL);

    // CL_DEVICE_MAX_CLOCK_FREQUENCY
    cl_uint clock_frequency;
    clGetDeviceInfo(cdDevices[current_device], CL_DEVICE_MAX_CLOCK_FREQUENCY, sizeof(clock_frequency), &clock_frequency, NULL);

	max_flops = compute_units * clock_frequency;
	++current_device;

	while( current_device < device_count )
	{
        // CL_DEVICE_MAX_COMPUTE_UNITS
        cl_uint compute_units;
        clGetDeviceInfo(cdDevices[current_device], CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(compute_units), &compute_units, NULL);

        // CL_DEVICE_MAX_CLOCK_FREQUENCY
        cl_uint clock_frequency;
        clGetDeviceInfo(cdDevices[current_device], CL_DEVICE_MAX_CLOCK_FREQUENCY, sizeof(clock_frequency), &clock_frequency, NULL);

        int flops = compute_units * clock_frequency;
		if( flops > max_flops )
		{
			max_flops        = flops;
			max_flops_device = cdDevices[current_device];
		}
		++current_device;
	}

    free(cdDevices);

	return max_flops_device;
}

///
/// Check for error condition and exit if found.  Print file and line number
/// of error. (from NVIDIA SDK)
///
void checkErrorFileLine(int errNum, int expected, const char* file, const int lineNumber)
{
    if (errNum != expected)
    {
        cerr << "Line " << lineNumber << " in File " << file << endl;
        exit(1);
    }
}


///
/// Round the local work size up to the next multiple of the size
///
int roundWorkSizeUp(int groupSize, int globalSize)
{
    int remainder = globalSize % groupSize;
    if (remainder == 0)
    {
        return globalSize;
    }
    else
    {
        return globalSize + groupSize - remainder;
    }
}
///////////////////////////////////////////////////////////////////////////////
//
//  Public Functions
//
//

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
                        int *outResultCosts, int numResults )
{
    // See what kind of devices are available
    cl_int errNum;
    cl_context cpuContext;
    cl_context gpuContext;

    // create the OpenCL context on available GPU devices
    gpuContext = clCreateContextFromType(0, CL_DEVICE_TYPE_GPU, NULL, NULL, &errNum);

    // Create an OpenCL context on available CPU devices
    cpuContext = clCreateContextFromType(0, CL_DEVICE_TYPE_CPU, NULL, NULL, &errNum);

    if (cpuContext == 0 && gpuContext == 0)
    {
        cerr << "ERROR: could not create any OpenCL context on CPU or GPU" << endl;
        return;
    }

    // For just a single result, just use multi-threaded CPU or single GPU
    if (numResults == 1)
    {
        if (gpuContext != 0)
        {
            cout << "Dijkstra OpenCL: Running single GPU version." << endl;
            runDijkstra(gpuContext, getMaxFlopsDev(gpuContext), graph, sourceVertices,
                        outResultCosts, numResults);
        }
        else
        {
            cout << "Dijkstra OpenCL: Running multithreaded CPU version." << endl;
            runDijkstra(cpuContext, getMaxFlopsDev(cpuContext), graph, sourceVertices,
                        outResultCosts, numResults);
        }
    }
    // For multiple results, prefer multi-GPU and fallback to CPU
    else
    {
        // Prefer Multi-GPU if multiple GPUs are available
        if (gpuContext != 0)
        {
            cout << "Dijkstra OpenCL: Running multi-GPU version." << endl;
            runDijkstraMultiGPU( gpuContext, graph, sourceVertices,
                                 outResultCosts, numResults );
        }
        // For now, fallback to CPU in this case.  I have a multi GPU+CPU path
        // but it does not seem to perform well because of the CPU overhead of
        // running the GPU version slows down the CPU version.
        else
        {
            cout << "Dijkstra OpenCL: Running multithreaded CPU version." << endl;
            runDijkstra(cpuContext, getMaxFlopsDev(cpuContext), graph, sourceVertices,
                        outResultCosts, numResults);
        }
    }

    clReleaseContext(cpuContext);
    clReleaseContext(gpuContext);
}

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
///                        each shortest path search will be written
/// \param numResults Should be the size of all three passed inarrays
///
void runDijkstra( cl_context context, cl_device_id deviceId, GraphData* graph,
                  int *sourceVertices, int *outResultCosts, int numResults)
{
    // Create command queue
    cl_int errNum;
    cl_command_queue commandQueue;
    commandQueue = clCreateCommandQueue( context, deviceId, 0, &errNum );
    checkError(errNum, CL_SUCCESS);

    // Program handle
    cl_program program = loadAndBuildProgram( context, "dijkstra.cl" );

    // Get the max workgroup size
    size_t maxWorkGroupSize;
    clGetDeviceInfo(deviceId, CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(size_t), &maxWorkGroupSize, NULL);

    cl_device_type dev_type;
    clGetDeviceInfo(deviceId, CL_DEVICE_TYPE, sizeof(dev_type), &dev_type, NULL);
    if (dev_type == CL_DEVICE_TYPE_CPU) {
        maxWorkGroupSize = 128;
    }
    checkError(errNum, CL_SUCCESS);
    cout << "MAX_WORKGROUP_SIZE: " << maxWorkGroupSize << endl;
    cout << "Computing '" << numResults << "' results." << endl;
    // Set # of work items in work group and total in 1 dimensional range
    size_t localWorkSize = maxWorkGroupSize;
    size_t globalWorkSize = roundWorkSizeUp(localWorkSize, graph->vertexCount);

    cl_mem vertexArrayDevice;
    cl_mem edgeArrayDevice;
    cl_mem weightArrayDevice;
    cl_mem maskArrayDevice;
    cl_mem costArrayDevice;
    cl_mem updatingCostArrayDevice;

    // Allocate buffers in Device memory
    allocateOCLBuffers( context, commandQueue, graph, &vertexArrayDevice, &edgeArrayDevice, &weightArrayDevice,
                        &maskArrayDevice, &costArrayDevice, &updatingCostArrayDevice, globalWorkSize);


    // Create the Kernels
    cl_kernel initializeBuffersKernel;
    initializeBuffersKernel = clCreateKernel(program, "initializeBuffers", &errNum);
    checkError(errNum, CL_SUCCESS);

    // Set the args values and check for errors
    errNum |= clSetKernelArg(initializeBuffersKernel, 0, sizeof(cl_mem), &maskArrayDevice);
    errNum |= clSetKernelArg(initializeBuffersKernel, 1, sizeof(cl_mem), &costArrayDevice);
    errNum |= clSetKernelArg(initializeBuffersKernel, 2, sizeof(cl_mem), &updatingCostArrayDevice);

    // 3 set below in loop
    errNum |= clSetKernelArg(initializeBuffersKernel, 4, sizeof(int), &graph->vertexCount);
    checkError(errNum, CL_SUCCESS);

    // Kernel 1
    cl_kernel ssspKernel1;
    ssspKernel1 = clCreateKernel(program, "OCL_SSSP_KERNEL1", &errNum);
    checkError(errNum, CL_SUCCESS);
    errNum |= clSetKernelArg(ssspKernel1, 0, sizeof(cl_mem), &vertexArrayDevice);
    errNum |= clSetKernelArg(ssspKernel1, 1, sizeof(cl_mem), &edgeArrayDevice);
    errNum |= clSetKernelArg(ssspKernel1, 2, sizeof(cl_mem), &weightArrayDevice);
    errNum |= clSetKernelArg(ssspKernel1, 3, sizeof(cl_mem), &maskArrayDevice);
    errNum |= clSetKernelArg(ssspKernel1, 4, sizeof(cl_mem), &costArrayDevice);
    errNum |= clSetKernelArg(ssspKernel1, 5, sizeof(cl_mem), &updatingCostArrayDevice);
    errNum |= clSetKernelArg(ssspKernel1, 6, sizeof(int), &graph->vertexCount);
    errNum |= clSetKernelArg(ssspKernel1, 7, sizeof(int), &graph->edgeCount);
    checkError(errNum, CL_SUCCESS);

    // Kernel 2
    cl_kernel ssspKernel2;
    ssspKernel2 = clCreateKernel(program, "OCL_SSSP_KERNEL2", &errNum);
    checkError(errNum, CL_SUCCESS);
    errNum |= clSetKernelArg(ssspKernel2, 0, sizeof(cl_mem), &vertexArrayDevice);
    errNum |= clSetKernelArg(ssspKernel2, 1, sizeof(cl_mem), &edgeArrayDevice);
    errNum |= clSetKernelArg(ssspKernel2, 2, sizeof(cl_mem), &weightArrayDevice);
    errNum |= clSetKernelArg(ssspKernel2, 3, sizeof(cl_mem), &maskArrayDevice);
    errNum |= clSetKernelArg(ssspKernel2, 4, sizeof(cl_mem), &costArrayDevice);
    errNum |= clSetKernelArg(ssspKernel2, 5, sizeof(cl_mem), &updatingCostArrayDevice);
    errNum |= clSetKernelArg(ssspKernel2, 6, sizeof(int), &graph->vertexCount);

    checkError(errNum, CL_SUCCESS);

    int *maskArrayHost = (int*) malloc(sizeof(int) * graph->vertexCount);

    for ( size_t i = 0 ; i < numResults; i++ )
    {

        errNum |= clSetKernelArg(initializeBuffersKernel, 3, sizeof(int), &sourceVertices[i]);
        checkError(errNum, CL_SUCCESS);

        // Initialize mask array to false, C and U to infiniti
        initializeOCLBuffers( commandQueue, initializeBuffersKernel, graph, maxWorkGroupSize );

        // Read mask array from device -> host
        cl_event readDone;
        errNum = clEnqueueReadBuffer( commandQueue, maskArrayDevice, CL_FALSE, 0, sizeof(int) * graph->vertexCount,
                                      maskArrayHost, 0, NULL, &readDone);
        checkError(errNum, CL_SUCCESS);
        clWaitForEvents(1, &readDone);

        while(!maskArrayEmpty(maskArrayHost, graph->vertexCount))
        {

            // In order to improve performance, we run some number of iterations
            // without reading the results.  This might result in running more iterations
            // than necessary at times, but it will in most cases be faster because
            // we are doing less stalling of the GPU waiting for results.
            for(int asyncIter = 0; asyncIter < NUM_ASYNCHRONOUS_ITERATIONS; asyncIter++)
            {
                size_t localWorkSize = maxWorkGroupSize;
                size_t globalWorkSize = roundWorkSizeUp(localWorkSize, graph->vertexCount);

                // execute the kernel
                errNum = clEnqueueNDRangeKernel(commandQueue, ssspKernel1, 1, 0, &globalWorkSize, &localWorkSize,
                                               0, NULL, NULL);
                checkError(errNum, CL_SUCCESS);

                errNum = clEnqueueNDRangeKernel(commandQueue, ssspKernel2, 1, 0, &globalWorkSize, &localWorkSize,
                                               0, NULL, NULL);
                checkError(errNum, CL_SUCCESS);
            }
            errNum = clEnqueueReadBuffer(commandQueue, maskArrayDevice, CL_FALSE, 0, sizeof(int) * graph->vertexCount,
                                         maskArrayHost, 0, NULL, &readDone);
            checkError(errNum, CL_SUCCESS);
            clWaitForEvents(1, &readDone);
        }

        // Copy the result back
        size_t offset = i * (size_t)graph->vertexCount;
        errNum = clEnqueueReadBuffer(commandQueue, costArrayDevice, CL_FALSE, 0, sizeof(int) * graph->vertexCount,
                                     &outResultCosts[offset], 0, NULL, &readDone);
        checkError(errNum, CL_SUCCESS);
        clWaitForEvents(1, &readDone);
    }

    free (maskArrayHost);

    clReleaseMemObject(vertexArrayDevice);
    clReleaseMemObject(edgeArrayDevice);
    clReleaseMemObject(weightArrayDevice);
    clReleaseMemObject(maskArrayDevice);
    clReleaseMemObject(costArrayDevice);
    clReleaseMemObject(updatingCostArrayDevice);

    clReleaseKernel(initializeBuffersKernel);
    clReleaseKernel(ssspKernel1);
    clReleaseKernel(ssspKernel2);

    clReleaseCommandQueue(commandQueue);
    clReleaseProgram(program);
    cout << "Computed '" << numResults << "' results" << endl;

}



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
/// \param endVertices Indices into the vertex array from which to end
///                    the search.
/// \param outResultsCosts A pre-allocated array where the results for
///                        each shortest path search will be written
/// \param numResults Should be the size of all three passed inarrays
///
///
void runDijkstraMultiGPU( cl_context gpuContext, GraphData* graph, int *sourceVertices,
                          int *outResultCosts, int numResults )
{

    // Find out how many GPU's to compute on all available GPUs
    cl_int errNum;
    size_t deviceBytes;
    cl_uint deviceCount;

    errNum = clGetContextInfo(gpuContext, CL_CONTEXT_DEVICES, 0, NULL, &deviceBytes);
    checkError(errNum, CL_SUCCESS);
    deviceCount = (cl_uint)deviceBytes/sizeof(cl_device_id);

    if (deviceCount == 0)
    {
        cerr << "ERROR: no GPUs present!" << endl;
        return;
    }

    DevicePlan *devicePlans = (DevicePlan*) malloc(sizeof(DevicePlan) * deviceCount);
    //pthread_t *threadIDs = (pthread_t*) malloc(sizeof(pthread_t) * deviceCount);

    boost::thread* bthread[deviceCount];

    // Divide the workload out per device
    int resultsPerDevice = numResults / deviceCount;

    size_t offset = 0;

    for (unsigned int i = 0; i < deviceCount; i++)
    {
        devicePlans[i].context = gpuContext;
        devicePlans[i].deviceId = getDev(gpuContext, i);;
        devicePlans[i].graph = graph;
        devicePlans[i].sourceVertices = &sourceVertices[offset];
        devicePlans[i].outResultCosts = &outResultCosts[offset * (size_t)graph->vertexCount];
        devicePlans[i].numResults = resultsPerDevice;

        offset += resultsPerDevice;
    }

    // Add any remaining work to the last GPU
    if (offset < numResults)
    {
        devicePlans[deviceCount - 1].numResults += (numResults - offset);
    }

    // Launch all the threads
    for (unsigned int i = 0; i < deviceCount; i++)
    {
        //pthread_create(&threadIDs[i], NULL, (void* (*)(void*))dijkstraThread, (void*)(devicePlans + i));
        bthread[i] = new boost::thread(&dijkstraThread, (DevicePlan*)(devicePlans + i));
    }

    // Wait for the results from all threads
    for (unsigned int i = 0; i < deviceCount; i++)
    {
        //pthread_join(threadIDs[i], NULL);
        bthread[i]->join();
    }

    for (unsigned int i = 0; i < deviceCount; i++)
    {
        delete bthread[i];
    }

    free (devicePlans);
    //free (threadIDs);
}

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
///                        each shortest path search will be written
/// \param numResults Should be the size of all three passed inarrays
///
///
void runDijkstraMultiGPUandCPU( const char* dir, cl_context gpuContext, cl_context cpuContext, GraphData* graph,
                                int *sourceVertices,
                                int *outResultCosts, int numResults )
{
    strcpy(cl_dir, dir);
    float ratioCPUtoGPU = 0.3; // CPU seems to run it at 2.26X on GT120 GPU


    // Find out how many GPU's to compute on all available GPUs
    cl_int errNum;
    size_t deviceBytes;
    cl_uint gpuDeviceCount;
    cl_uint cpuDeviceCount;

    errNum = clGetContextInfo(gpuContext, CL_CONTEXT_DEVICES, 0, NULL, &deviceBytes);
    checkError(errNum, CL_SUCCESS);
    gpuDeviceCount = (cl_uint)deviceBytes/sizeof(cl_device_id);

    if (gpuDeviceCount == 0) {
        cerr << "ERROR: no GPUs present!" << endl;
        return;
    }

    cl_uint totalDeviceCount = gpuDeviceCount + 1;

    DevicePlan *devicePlans = (DevicePlan*) malloc(sizeof(DevicePlan) * totalDeviceCount);
    boost::thread* bthread[totalDeviceCount];

    int cpuResults = numResults * (ratioCPUtoGPU);
    cout << "cpuResults: " << cpuResults << endl;

    int gpuResults = numResults - cpuResults;
    cout << "gpuResults: " << gpuResults << endl;

    // run on GPUs:
    // Divide the workload out per device
    int resultsPerGPU = gpuResults / gpuDeviceCount;
    int curDevice = 0;
    int offset = cpuResults;
    for (unsigned int i = 0; i < gpuDeviceCount; i++) {
        devicePlans[curDevice].context = gpuContext;
        devicePlans[curDevice].deviceId = getDev(gpuContext, i);;
        devicePlans[curDevice].graph = graph;
        devicePlans[curDevice].sourceVertices = &sourceVertices[offset];
        devicePlans[curDevice].outResultCosts = &outResultCosts[(size_t)offset * (size_t)graph->vertexCount];
        devicePlans[curDevice].numResults = resultsPerGPU;

        offset += resultsPerGPU;
        curDevice++;
    }

    // Add any remaining work to the last GPU
    if (offset < numResults) {
        devicePlans[gpuDeviceCount - 1].numResults += (numResults - offset);
    }

    // Launch all the threads
    for (unsigned int i = 0; i < gpuDeviceCount; i++) {
        bthread[i] = new boost::thread(&dijkstraThread, (DevicePlan*)(devicePlans + i));
    }
    // run on CPU
    bthread[totalDeviceCount-1] = new boost::thread(
            boost::bind(&runDijkstraMT, graph, sourceVertices, outResultCosts, cpuResults));

    // Wait for the results from all threads
    for (unsigned int i = 0; i < totalDeviceCount; i++) {
        bthread[i]->join();
    }

    for (unsigned int i = 0; i < totalDeviceCount; i++) {
        delete bthread[i];
    }
    free (devicePlans);
}

void runDijkstraCPUOnly( const char* dir, GraphData* graph,
                               int *sourceVertices,
                               int *outResultCosts, int numResults )
{
    strcpy(cl_dir, dir);

}
///
/// Check whether the mask array is empty.  This tells the algorithm whether
/// it needs to continue running or not.
///
bool maskArrayEmpty(int *maskArray, int count)
{
    for(int i = 0; i < count; i++ )
    {
        if (maskArray[i] == 1)
        {
            return false;
        }
    }

    return true;
}

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
void runDijkstraRef( GraphData* graph, int *sourceVertices,
                     int *outResultCosts, int start, int end)
{

    // Create the arrays needed for processing the algorithm
    int *costArray = new int [graph->vertexCount];
    int *updatingCostArray = new int[graph->vertexCount];
    int *maskArray = new int[graph->vertexCount];

    for (size_t i = start; i <= end; i++)
    {
        // Initialize the buffer for this run
        for (int v = 0; v < graph->vertexCount; v++)
        {
            if (v == sourceVertices[i])
            {
                maskArray[v] = 1;
                costArray[v] = 0;
                updatingCostArray[v] = 0;
            }
            else
            {
                maskArray[v] = 0;
                costArray[v] = INT_MAX;
                updatingCostArray[v] = INT_MAX;
            }
        }

        while(!maskArrayEmpty(maskArray, graph->vertexCount))
        {
            // Equivalent of OCL_SSSP_KERNEL1()
            for (int tid = 0; tid < graph->vertexCount; tid++)
            {
                if ( maskArray[tid] != 0 )
                {
                    maskArray[tid] = 0;

                    int edgeStart = graph->vertexArray[tid];
                    int edgeEnd;
                    if (tid + 1 < (graph->vertexCount))
                    {
                        edgeEnd = graph->vertexArray[tid + 1];
                    }
                    else
                    {
                        edgeEnd = graph->edgeCount;
                    }

                    for(int edge = edgeStart; edge < edgeEnd; edge++)
                    {
                        int nid = graph->edgeArray[edge];

                        // One note here: whereas the paper specified weightArray[nid], I
                        //  found that the correct thing to do was weightArray[edge].  I think
                        //  this was a typo in the paper.  Either that, or I misunderstood
                        //  the data structure.
                        if (updatingCostArray[nid] > (costArray[tid] + graph->weightArray[edge]))
                        {
                            updatingCostArray[nid] = (costArray[tid] + graph->weightArray[edge]);
                        }
                    }
                }
            }

            // Equivalent of OCL_SSSP_KERNEL2()
            for (int tid = 0; tid < graph->vertexCount; tid++)
            {
                if (costArray[tid] > updatingCostArray[tid])
                {
                    costArray[tid] = updatingCostArray[tid];
                    maskArray[tid] = 1;
                }

                updatingCostArray[tid] = costArray[tid];
            }
        }

        // Copy the result back
        memcpy(&outResultCosts[i * (size_t)graph->vertexCount], costArray, sizeof(int) * graph->vertexCount);
    }

    // Free temporary computation buffers
    delete [] costArray;
    delete [] updatingCostArray;
    delete [] maskArray;
}

void runDijkstraMT(GraphData* graph, int *sourceVertices,
                   int *outResultCosts, int numResults )
{
    cout << "CPU: Computing '" << numResults << "' results." << endl;
    unsigned int nCPUs = boost::thread::hardware_concurrency();
    int work_chunk = numResults / nCPUs;

    if (work_chunk == 0) work_chunk = 1;

    int obs_start = 0;
    int obs_end = obs_start + work_chunk;
    int quotient = numResults / nCPUs;
    int remainder = numResults % nCPUs;
    int tot_threads = (quotient > 0) ? nCPUs : remainder;

    boost::thread* bthread[tot_threads];

    for (unsigned int i=0; i<tot_threads; i++) {
        int a=0;
        int b=0;
        if (i < remainder) {
            a = i*(quotient+1);
            b = a+quotient;
        } else {
            a = remainder*(quotient+1) + (i-remainder)*quotient;
            b = a+quotient-1;
        }

        bthread[i] = new boost::thread(boost::bind(&runDijkstraRef, (GraphData*) graph,
                (int*)sourceVertices, (int*)outResultCosts, a, b));
    }
    for (unsigned int i = 0; i < tot_threads; i++) {
        bthread[i]->join();
    }

    for (unsigned int i = 0; i < tot_threads; i++) {
        delete bthread[i];
    }
    cout << "CPU: Computed '" << numResults << "' results." << endl;
}


void ssspThread(DevicePlan *plan)
{
    //runSSSP( plan->context, plan->deviceId, plan->graph, plan->sourceVertices,
    //            plan->outResultCosts, plan->numResults );
}

void runSSSPGPU(const char* dir, cl_context gpuContext, cl_context cpuContext, GraphData* graph,
                int *sourceVertices, int *outResultCosts, int numResults )
{
    strcpy(cl_dir, dir);
    float ratioCPUtoGPU = 0.45; // CPU seems to run it at 2.26X on GT120 GPU

    // Find out how many GPU's to compute on all available GPUs
    cl_int errNum;
    size_t deviceBytes;
    cl_uint gpuDeviceCount;
    cl_uint cpuDeviceCount;

    errNum = clGetContextInfo(gpuContext, CL_CONTEXT_DEVICES, 0, NULL, &deviceBytes);
    checkError(errNum, CL_SUCCESS);
    gpuDeviceCount = (cl_uint)deviceBytes/sizeof(cl_device_id);

    if (gpuDeviceCount == 0) {
        cerr << "ERROR: no GPUs present!" << endl;
        return;
    }

    cl_uint totalDeviceCount = gpuDeviceCount + 1;

    DevicePlan *devicePlans = (DevicePlan*) malloc(sizeof(DevicePlan) * totalDeviceCount);
    boost::thread* bthread[totalDeviceCount];

    int cpuResults = numResults * (ratioCPUtoGPU);
    cout << "cpuResults: " << cpuResults << endl;

    int gpuResults = numResults - cpuResults;
    cout << "gpuResults: " << gpuResults << endl;

    // run on GPUs:
    // Divide the workload out per device
    int resultsPerGPU = gpuResults / gpuDeviceCount;
    int curDevice = 0;
    int offset = cpuResults;
    for (unsigned int i = 0; i < gpuDeviceCount; i++) {
        devicePlans[curDevice].context = gpuContext;
        devicePlans[curDevice].deviceId = getDev(gpuContext, i);;
        devicePlans[curDevice].graph = graph;
        devicePlans[curDevice].sourceVertices = &sourceVertices[offset];
        devicePlans[curDevice].outResultCosts = &outResultCosts[(size_t)offset * (size_t)graph->vertexCount];
        devicePlans[curDevice].numResults = resultsPerGPU;

        offset += resultsPerGPU;
        curDevice++;
    }

    // Add any remaining work to the last GPU
    if (offset < numResults) {
        devicePlans[gpuDeviceCount - 1].numResults += (numResults - offset);
    }

    // Launch all the threads
    for (unsigned int i = 0; i < gpuDeviceCount; i++) {
        bthread[i] = new boost::thread(&ssspThread, (DevicePlan*)(devicePlans + i));
    }
    // run on CPU
    bthread[totalDeviceCount-1] = new boost::thread(
                                                    boost::bind(&runDijkstraMT, graph, sourceVertices, outResultCosts, cpuResults));

    // Wait for the results from all threads
    for (unsigned int i = 0; i < totalDeviceCount; i++) {
        bthread[i]->join();
    }

    for (unsigned int i = 0; i < totalDeviceCount; i++) {
        delete bthread[i];
    }
    free (devicePlans);
}

char *replace_str1(char *str, char *orig, char *rep, int start)
{
    static char temp[4096];
    static char buffer[4096];
    char *p;

    strcpy(temp, str + start);

    if(!(p = strstr(temp, orig)))  // Is 'orig' even in 'temp'?
        return temp;

    strncpy(buffer, temp, p-temp); // Copy characters from 'temp' start to 'orig' str
    buffer[p-temp] = '\0';

    sprintf(buffer + (p - temp), "%s%s", rep, p + strlen(orig));
    sprintf(str + start, "%s", buffer);

    return str;
}

void runSSSP( const char* dir, cl_context context, cl_device_id deviceId, GraphData* graph,
                 int *sourceVertices, int *outResultCosts, int numResults)
{
    strcpy(cl_dir, dir);

    size_t results_size = (size_t)graph->vertexCount * (size_t)numResults;
    // Create command queue
    cl_int errNum;
    cl_command_queue commandQueue;
    commandQueue = clCreateCommandQueue( context, deviceId, 0, &errNum );
    checkError(errNum, CL_SUCCESS);

    // Program handle
    cl_program program = loadAndBuildProgram( context, "sssp.cl" );
/*
    char *source_str;
    size_t source_size;

    char file_path[1024];
    strncpy(file_path, cl_dir, 1024);
    strcat(file_path, "sssp.cl");
    FILE *fp = fopen(file_path, "r");
    source_str = (char*)malloc(MAX_SOURCE_SIZE);
    source_size = fread( source_str, 1, MAX_SOURCE_SIZE, fp);
    fclose(fp);

    char msg[25];
#ifdef __WIN32__
    _snprintf(msg, sizeof(msg), "%d", graph->vertexCount);
#else
    snprintf(msg, sizeof(msg), "%d", graph->vertexCount);
#endif
    replace_str1(source_str, "7777", msg, 0);
    replace_str1(source_str, "8888", msg, 0);
    replace_str1(source_str, "9999", msg, 0);
    source_size = strlen(source_str);
    cl_program program = clCreateProgramWithSource(context, 1, (const char **)&source_str, (const size_t *)&source_size, &errNum);
*/
    // Get the max workgroup size
    size_t maxWorkGroupSize;
    clGetDeviceInfo(deviceId, CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(size_t), &maxWorkGroupSize, NULL);

    cl_device_type dev_type;
    clGetDeviceInfo(deviceId, CL_DEVICE_TYPE, sizeof(dev_type), &dev_type, NULL);
    if (dev_type == CL_DEVICE_TYPE_CPU) {
        maxWorkGroupSize = 128;
    }
    checkError(errNum, CL_SUCCESS);
    cout << "MAX_WORKGROUP_SIZE: " << maxWorkGroupSize << endl;
    cout << "Computing '" << numResults << "' results." << endl;
    // Set # of work items in work group and total in 1 dimensional range
    size_t localWorkSize = maxWorkGroupSize;
    size_t globalWorkSize = roundWorkSizeUp(localWorkSize, graph->vertexCount);

    cl_mem vertexArrayDevice;
    cl_mem edgeArrayDevice;
    cl_mem weightArrayDevice;
    cl_mem sourceArrayDevice;
    cl_mem maskArrayDevice;
    cl_mem costArrayDevice;
    cl_mem updatingCostArrayDevice;
    cl_mem resultArrayDevice;

    // Allocate buffers in Device memory
    cl_mem hostVertexArrayBuffer;
    cl_mem hostEdgeArrayBuffer;
    cl_mem hostWeightArrayBuffer;
    cl_mem hostSourceArrayBuffer;

    // First, need to create OpenCL Host buffers that can be copied to device buffers
    hostVertexArrayBuffer = clCreateBuffer(context, CL_MEM_COPY_HOST_PTR | CL_MEM_ALLOC_HOST_PTR,
                                           sizeof(int) * graph->vertexCount, graph->vertexArray, &errNum);
    checkError(errNum, CL_SUCCESS);
    hostEdgeArrayBuffer = clCreateBuffer(context, CL_MEM_COPY_HOST_PTR | CL_MEM_ALLOC_HOST_PTR,
                                         sizeof(int) * graph->edgeCount, graph->edgeArray, &errNum);
    checkError(errNum, CL_SUCCESS);
    hostWeightArrayBuffer = clCreateBuffer(context, CL_MEM_COPY_HOST_PTR | CL_MEM_ALLOC_HOST_PTR,
                                           sizeof(int) * graph->edgeCount, graph->weightArray, &errNum);
    checkError(errNum, CL_SUCCESS);
    hostSourceArrayBuffer = clCreateBuffer(context, CL_MEM_COPY_HOST_PTR | CL_MEM_ALLOC_HOST_PTR,
                                           sizeof(int) * numResults, sourceVertices, &errNum);
    checkError(errNum, CL_SUCCESS);
    // Now create all of the GPU buffers
    vertexArrayDevice = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(int) * globalWorkSize, NULL, &errNum);
    checkError(errNum, CL_SUCCESS);
    edgeArrayDevice = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(int) * graph->edgeCount, NULL, &errNum);
    checkError(errNum, CL_SUCCESS);
    weightArrayDevice = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(int) * graph->edgeCount, NULL, &errNum);
    checkError(errNum, CL_SUCCESS);
    sourceArrayDevice = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(int) * numResults, NULL, &errNum);
    checkError(errNum, CL_SUCCESS);
    maskArrayDevice = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(int) * globalWorkSize, NULL, &errNum);
    checkError(errNum, CL_SUCCESS);
    costArrayDevice = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(int) * globalWorkSize, NULL, &errNum);
    checkError(errNum, CL_SUCCESS);
    updatingCostArrayDevice = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(int) * globalWorkSize, NULL, &errNum);
    checkError(errNum, CL_SUCCESS);
    resultArrayDevice = clCreateBuffer(context, CL_MEM_READ_WRITE,
                                       sizeof(int) * results_size, NULL, &errNum);
    checkError(errNum, CL_SUCCESS);

    // Now queue up the data to be copied to the device
    errNum = clEnqueueCopyBuffer(commandQueue, hostVertexArrayBuffer, vertexArrayDevice, 0, 0,
                                 sizeof(int) * graph->vertexCount, 0, NULL, NULL);
    checkError(errNum, CL_SUCCESS);
    errNum = clEnqueueCopyBuffer(commandQueue, hostEdgeArrayBuffer, edgeArrayDevice, 0, 0,
                                 sizeof(int) * graph->edgeCount, 0, NULL, NULL);
    checkError(errNum, CL_SUCCESS);
    errNum = clEnqueueCopyBuffer(commandQueue, hostWeightArrayBuffer, weightArrayDevice, 0, 0,
                                 sizeof(int) * graph->edgeCount, 0, NULL, NULL);
    checkError(errNum, CL_SUCCESS);
    errNum = clEnqueueCopyBuffer(commandQueue, hostSourceArrayBuffer, sourceArrayDevice, 0, 0,
                                 sizeof(int) * numResults, 0, NULL, NULL);
    checkError(errNum, CL_SUCCESS);

    clReleaseMemObject(hostVertexArrayBuffer);
    clReleaseMemObject(hostEdgeArrayBuffer);
    clReleaseMemObject(hostWeightArrayBuffer);
    clReleaseMemObject(hostSourceArrayBuffer);

    // Create the Kernels
    cl_kernel ssspKernel1;
    ssspKernel1 = clCreateKernel(program, "SSSP_KERNEL", &errNum);
    checkError(errNum, CL_SUCCESS);
    errNum |= clSetKernelArg(ssspKernel1, 0, sizeof(cl_int), &graph->vertexCount);
    errNum |= clSetKernelArg(ssspKernel1, 1, sizeof(cl_int), &graph->edgeCount);
    errNum |= clSetKernelArg(ssspKernel1, 2, sizeof(cl_mem), &vertexArrayDevice);
    errNum |= clSetKernelArg(ssspKernel1, 3, sizeof(cl_mem), &edgeArrayDevice);
    errNum |= clSetKernelArg(ssspKernel1, 4, sizeof(cl_mem), &weightArrayDevice);
    errNum |= clSetKernelArg(ssspKernel1, 5, sizeof(cl_mem), &maskArrayDevice);
    errNum |= clSetKernelArg(ssspKernel1, 6, sizeof(cl_mem), &costArrayDevice);
    errNum |= clSetKernelArg(ssspKernel1, 7, sizeof(cl_mem), &updatingCostArrayDevice);
    errNum |= clSetKernelArg(ssspKernel1, 8, sizeof(cl_mem), &sourceArrayDevice);
    errNum |= clSetKernelArg(ssspKernel1, 9, sizeof(cl_mem), &resultArrayDevice);

    checkError(errNum, CL_SUCCESS);

    errNum = clEnqueueNDRangeKernel(commandQueue, ssspKernel1, 1, 0,
                                    &globalWorkSize, &localWorkSize,
                                    0, NULL, NULL);
    checkError(errNum, CL_SUCCESS);

    // Copy the result back
    cl_event readDone;
    size_t offset = 0;
    errNum = clEnqueueReadBuffer(commandQueue, resultArrayDevice, CL_FALSE, 0,
                                 sizeof(int) * graph->vertexCount,
                                 &outResultCosts[offset], 0, NULL, &readDone);
    checkError(errNum, CL_SUCCESS);
    clWaitForEvents(1, &readDone);

    clReleaseMemObject(vertexArrayDevice);
    clReleaseMemObject(edgeArrayDevice);
    clReleaseMemObject(weightArrayDevice);
    clReleaseMemObject(resultArrayDevice);

    clReleaseKernel(ssspKernel1);

    clReleaseCommandQueue(commandQueue);
    clReleaseProgram(program);
    cout << "Computed '" << numResults << "' results" << endl;

}





//////////////////

// A utility function to create a new adjacency list node
struct AdjListNode* newAdjListNode(int dest, int weight)
{
    struct AdjListNode* newNode =
    (struct AdjListNode*) malloc(sizeof(struct AdjListNode));
    newNode->dest = dest;
    newNode->weight = weight;
    newNode->next = NULL;
    return newNode;
}

// A utility function that creates a graph of V vertices
struct Graph* createGraph(int V)
{
    struct Graph* graph = (struct Graph*) malloc(sizeof(struct Graph));
    graph->V = V;

    // Create an array of adjacency lists.  Size of array will be V
    graph->array = (struct AdjList*) malloc(V * sizeof(struct AdjList));

    // Initialize each adjacency list as empty by making head as NULL
    for (int i = 0; i < V; ++i)
        graph->array[i].head = NULL;

    return graph;
}

// Adds an edge to an undirected graph
void addEdgeToGraph(struct Graph* graph, int src, int dest, int weight)
{
    // Add an edge from src to dest.  A new node is added to the adjacency
    // list of src.  The node is added at the begining
    struct AdjListNode* newNode = newAdjListNode(dest, weight);
    newNode->next = graph->array[src].head;
    graph->array[src].head = newNode;

    // Since graph is undirected, add an edge from dest to src also
    newNode = newAdjListNode(src, weight);
    newNode->next = graph->array[dest].head;
    graph->array[dest].head = newNode;
}

// A utility function to create a new Min Heap Node
struct MinHeapNode* newMinHeapNode(int v, int dist)
{
    MinHeapNode* minHeapNode = new MinHeapNode();
        //(struct MinHeapNode*) malloc(sizeof(struct MinHeapNode));
    minHeapNode->v = v;
    minHeapNode->dist = dist;
    return minHeapNode;
}

// A utility function to create a Min Heap
struct MinHeap* createMinHeap(int capacity)
{
    MinHeap* minHeap = new MinHeap();
    minHeap->pos = new int[capacity];
    minHeap->size = 0;
    minHeap->capacity = capacity;
    minHeap->array = new MinHeapNode*[capacity];
        //(struct MinHeapNode**) malloc(capacity * sizeof(struct MinHeapNode*));
    return minHeap;
}

/* free a minHeap struct*/
void freeMinheap(MinHeap* minHeap) {
    free(minHeap->pos);
    free(minHeap->array);
    free(minHeap);
}

// A utility function to swap two nodes of min heap. Needed for min heapify
void swapMinHeapNode(struct MinHeapNode** a, struct MinHeapNode** b)
{
    struct MinHeapNode t;
    t.v = (*a)->v;
    t.dist = (*a)->dist;

    (*a)->v = (*b)->v;
    (*a)->dist = (*b)->dist;

    (*b)->v = t.v;
    (*b)->dist = t.dist;
}

// A standard function to heapify at given idx
// This function also updates position of nodes when they are swapped.
// Position is needed for decreaseKey()
void minHeapify(struct MinHeap* minHeap, int idx)
{
    int smallest, left, right;
    smallest = idx;
    left = 2 * idx + 1;
    right = 2 * idx + 2;

    if (left < minHeap->size &&
        minHeap->array[left]->dist < minHeap->array[smallest]->dist )
        smallest = left;

    if (right < minHeap->size &&
        minHeap->array[right]->dist < minHeap->array[smallest]->dist )
        smallest = right;

    if (smallest != idx)
        {
        // The nodes to be swapped in min heap
        MinHeapNode *smallestNode = minHeap->array[smallest];
        MinHeapNode *idxNode = minHeap->array[idx];

        // Swap positions
        minHeap->pos[smallestNode->v] = idx;
        minHeap->pos[idxNode->v] = smallest;

        // Swap nodes
        swapMinHeapNode(&minHeap->array[smallest], &minHeap->array[idx]);

        minHeapify(minHeap, smallest);
        }
}
// A utility function to check if the given minHeap is ampty or not
int isEmpty(struct MinHeap* minHeap)
{
    return minHeap->size == 0;
}

// Standard function to extract minimum node from heap
struct MinHeapNode* extractMin(struct MinHeap* minHeap)
{
    if (isEmpty(minHeap))
        return NULL;

    // Store the root node
    struct MinHeapNode* root = minHeap->array[0];

    // Replace root node with last node
    struct MinHeapNode* lastNode = minHeap->array[minHeap->size - 1];
    minHeap->array[0] = lastNode;

    // Update position of last node
    minHeap->pos[root->v] = minHeap->size-1;
    minHeap->pos[lastNode->v] = 0;

    // Reduce heap size and heapify root
    --minHeap->size;
    minHeapify(minHeap, 0);

    return root;
}

// Function to decreasy dist value of a given vertex v. This function
// uses pos[] of min heap to get the current index of node in min heap
void decreaseKey(struct MinHeap* minHeap, int v, int dist)
{
    // Get the index of v in  heap array
    int i = minHeap->pos[v];

    // Get the node and update its dist value
    minHeap->array[i]->dist = dist;

    // Travel up while the complete tree is not hepified.
    // This is a O(Logn) loop
    while (i && minHeap->array[i]->dist < minHeap->array[(i - 1) / 2]->dist)
        {
        // Swap this node with its parent
        minHeap->pos[minHeap->array[i]->v] = (i-1)/2;
        minHeap->pos[minHeap->array[(i-1)/2]->v] = i;
        swapMinHeapNode(&minHeap->array[i],  &minHeap->array[(i - 1) / 2]);

        // move to parent index
        i = (i - 1) / 2;
        }
}

// A utility function to check if a given vertex
// 'v' is in min heap or not
bool isInMinHeap(struct MinHeap *minHeap, int v)
{
    if (minHeap->pos[v] < minHeap->size)
        return true;
    return false;
}

// A utility function used to print the solution
void printArr(int dist[], int n)
{
    printf("Vertex   Distance from Source\n");
    for (int i = 0; i < n; ++i)
        printf("%d \t\t %d\n", i, dist[i]);
}

// The main function that calulates distances of shortest paths from src to all
// vertices. It is a O(ELogV) function
void dijkstra(struct Graph* graph, int src, int i_result, int* results)
{
    int V = graph->V;// Get the number of vertices in graph
    int dist[V];      // dist values used to pick minimum weight edge in cut

    // minHeap represents set E
    struct MinHeap* minHeap = createMinHeap(V);

    // Initialize min heap with all vertices. dist value of all vertices
    for (int v = 0; v < V; ++v)
        {
        dist[v] = INT_MAX;
        minHeap->array[v] = newMinHeapNode(v, dist[v]);
        minHeap->pos[v] = v;
        }

    // Make dist value of src vertex as 0 so that it is extracted first
    minHeap->array[src] = newMinHeapNode(src, dist[src]);
    minHeap->pos[src]   = src;
    dist[src] = 0;
    decreaseKey(minHeap, src, dist[src]);

    // Initially size of min heap is equal to V
    minHeap->size = V;

    // In the followin loop, min heap contains all nodes
    // whose shortest distance is not yet finalized.
    while (!isEmpty(minHeap))
        {
        // Extract the vertex with minimum distance value
        struct MinHeapNode* minHeapNode = extractMin(minHeap);
        int u = minHeapNode->v; // Store the extracted vertex number

        // Traverse through all adjacent vertices of u (the extracted
        // vertex) and update their distance values
        struct AdjListNode* pCrawl = graph->array[u].head;
        while (pCrawl != NULL)
            {
            int v = pCrawl->dest;

            // If shortest distance to v is not finalized yet, and distance to v
            // through u is less than its previously calculated distance
            if (isInMinHeap(minHeap, v) && dist[u] != INT_MAX &&
                pCrawl->weight + dist[u] < dist[v])
                {
                dist[v] = dist[u] + pCrawl->weight;

                // update distance value in min heap also
                decreaseKey(minHeap, v, dist[v]);
                }
            pCrawl = pCrawl->next;
            }
        }

    //free allocated memory
    delete[] minHeap->pos;
    for (int i=1; i<minHeap->capacity; i++) delete minHeap->array[i];
    delete[] minHeap->array;
    delete minHeap;

    // print the calculated shortest distances
    //printArr(dist, V);
    
    //memcpy(&results[offset], &dist, sizeof(int) * graph->V);

    //for (int i = 0; i < V; ++i) {
        //printf("%d \t\t %d\n", i, dist[i]);
    //    results[offset+i] = dist[i];
    //}
}

void freeGraph(Graph* graph)
{
    struct AdjListNode *AdjListNode;
    struct AdjListNode *temp;
    for (int i = 0; i < graph->V; i++) {
        if (graph->array[i].head) {
            AdjListNode = graph->array[i].head;
            while (AdjListNode) {
                temp = AdjListNode->next;
                free(AdjListNode);
                AdjListNode = temp;
            }

        }
    }
    free(graph->array);
    free(graph);
}
