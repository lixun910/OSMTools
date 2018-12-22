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



# Main project
add_executable(OSMToolsProject Test.cpp Downloader.cpp Downloader.h Roads.cpp Roads.h)

include_directories(${CURL_INCLUDE_DIR})
target_link_libraries(OSMToolsProject ${CURL_LIBRARIES})
include_directories( ${Boost_INCLUDE_DIR} )
target_link_libraries(OSMToolsProject ${Boost_LIBRARIES})
include_directories(${GDAL_INCLUDE_DIR})
target_link_libraries(OSMToolsProject ${GDAL_LIBRARIES})
