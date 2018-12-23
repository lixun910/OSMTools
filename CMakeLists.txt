cmake_minimum_required(VERSION 2.8)

project(OSMToolsProject)

set (CMAKE_CXX_STANDARD 11)
set(DEP_LIBRARY_PATH ${CMAKE_SOURCE_DIR}/deps)

# Dependencies:
# libcurl
set(CURL_LIBRARY "-lcurl")
find_package(CURL REQUIRED)
# boost
find_package(Boost COMPONENTS thread system REQUIRED)
# gdal
find_package(GDAL REQUIRED)
# ann
set (ANN_INCLUDE_DIR ${DEP_LIBRARY_PATH}/ANN/include)
set (ANN_LIBRARIES ${DEP_LIBRARY_PATH}/ANN/lib/libANN.a)
# opencl
if (APPLE)
    # osx > 10.13 deprecates OpenCL
    set(CMAKE_OSX_SYSROOT /Applications/Xcode9.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.13.sdk)
    set(OPENCL_LIBRARIES "-framework OpenCL")
endif()


# Main project
add_executable(OSMToolsProject Test.cpp csv.h oclDijkstraKernel.cpp oclDijkstraKernel.h Downloader.cpp Downloader.h
Roads.cpp Roads.h TravelTool.cpp TravelTool.h)

include_directories(${CURL_INCLUDE_DIR})
target_link_libraries(OSMToolsProject ${CURL_LIBRARIES})

include_directories( ${Boost_INCLUDE_DIR} )
target_link_libraries(OSMToolsProject ${Boost_LIBRARIES})

include_directories(${GDAL_INCLUDE_DIR})
target_link_libraries(OSMToolsProject ${GDAL_LIBRARIES})

include_directories(${ANN_INCLUDE_DIR})
target_link_libraries(OSMToolsProject ${ANN_LIBRARIES})

target_link_libraries(OSMToolsProject ${OPENCL_LIBRARIES})
configure_file(dijkstra.cl ${CMAKE_CURRENT_BINARY_DIR}/dijkstra.cl COPYONLY)
