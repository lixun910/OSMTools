cmake_minimum_required(VERSION 2.8)

project(OSMToolsProject)

set (CMAKE_CXX_STANDARD 11)
set(DEP_LIBRARY_PATH ${CMAKE_SOURCE_DIR}/deps)

# Dependencies:
# libcurl
set(CURL_LIBRARY "-lcurl")
find_package(CURL REQUIRED)
# boost
find_package(Boost REQUIRED)
# gdal
find_package(GDAL REQUIRED)

# json_spirit
set(JSON_SPIRIT_DIR ${DEP_LIBRARY_PATH}/json_spirit_v4.08)
set(JSON_SPIRIT_INCLUDE_DIR ${JSON_SPIRIT_DIR}/include)
if (UNIX)
    set(JSON_SPIRIT_LIBRARIES ${JSON_SPIRIT_DIR}/lib/libjson_spirit.a)
endif()
if (WIN32)
    set(JSON_SPIRIT_LIBRARIES ${JSON_SPIRIT_DIR}/lib/libjson_spirit_win32.a)
endif()
if (APPLE)
    set(JSON_SPIRIT_LIBRARIES ${JSON_SPIRIT_DIR}/lib/libjson_spirit_osx.a)
endif()

# Main project
add_executable(OSMToolsProject Test.cpp Downloader.cpp Downloader.h Roads.cpp Roads.h)

include_directories(${CURL_INCLUDE_DIR})
target_link_libraries(OSMToolsProject ${CURL_LIBRARIES})
include_directories( ${Boost_INCLUDE_DIR} )
target_link_libraries(OSMToolsProject ${Boost_LIBRARIES})
include_directories(${JSON_SPIRIT_INCLUDE_DIR})
target_link_libraries(OSMToolsProject ${JSON_SPIRIT_LIBRARIES})
include_directories(${GDAL_INCLUDE_DIR})
target_link_libraries(OSMToolsProject ${GDAL_LIBRARIES})
