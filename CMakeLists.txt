cmake_minimum_required(VERSION 2.8)

project(OSMToolsProject)

set (CMAKE_CXX_STANDARD 11)

set (DEP_LIBRARY_PATH ${CMAKE_SOURCE_DIR}/deps)

# Dependencies:
# libcurl
find_package(CURL REQUIRED)
# boost
find_package(Boost COMPONENTS thread system REQUIRED)
# gdal
find_package(GDAL REQUIRED)
set (GDAL_INCLUDE_DIR "/usr/local/include")
set (GDAL_LIBRARIES "-L/usr/local/lib -lgdal")
message(WARNING ${GDAL_INCLUDE_DIR})
# wxWidgets
set (wxWidgets_CONFIGURATION /usr/local/bin/wx-config)
find_package(wxWidgets REQUIRED core base REQUIRED)
# ann
set (ANN_INCLUDE_DIR ${DEP_LIBRARY_PATH}/ANN/include)
set (ANN_LIBRARIES ${DEP_LIBRARY_PATH}/ANN/lib/libANN.a)

# Main project
add_executable(OSMToolsProject
        Test.cpp Test.h
        Downloader.cpp Downloader.h
        Roads.cpp Roads.h
        uiRoadDownload.cpp uiRoadDownload.h
        OGRDataUtils.cpp OGRDataUtils.h
        OGRShapeUtils.cpp OGRShapeUtils.h
        )

include_directories(${CURL_INCLUDE_DIR})
target_link_libraries(OSMToolsProject ${CURL_LIBRARIES})

include_directories( ${Boost_INCLUDE_DIR} )
target_link_libraries(OSMToolsProject ${Boost_LIBRARIES})

include_directories(${GDAL_INCLUDE_DIR})
target_link_libraries(OSMToolsProject ${GDAL_LIBRARIES})

include_directories(${ANN_INCLUDE_DIR})
target_link_libraries(OSMToolsProject ${ANN_LIBRARIES})

include(${wxWidgets_USE_FILE})
include_directories(${wxWidgets_INCLUDE_DIRS})
target_link_libraries(OSMToolsProject ${wxWidgets_LIBRARIES})
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${wxWidgets_CXX_FLAGS}")
