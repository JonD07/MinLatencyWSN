#
# This very appreciated template was provided by Matthias Miltenberger.
# To build:
#  > cmake -H. -Bbuild
#  > cmake --build build
#

cmake_minimum_required(VERSION 3.10)

project(min-lat)

option(CXX "enable C++ compilation" ON)
if(CXX)
	enable_language(CXX)
endif()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "debug")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
set(CMAKE_CXX_FLAGS "-O0 -ggdb -g -Wall -Werror")

# Visual Studio compiler with static runtime libraries
if(MSVC AND MT)
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT -O0 -ggdb -g -Wall -Werror")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd -O0 -ggdb -g -Wall -Werror")
	set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT -O0 -ggdb -g -Wall -Werror")
	set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MTd -O0 -ggdb -g -Wall -Werror")
endif()

find_package(GUROBI REQUIRED)

include_directories(
	${GUROBI_INCLUDE_DIRS}
	${CMAKE_CURRENT_LIST_DIR}/inc
)

# list source files here
set(sources src/main.cpp)

add_executable(${CMAKE_PROJECT_NAME}
	${sources}
	src/Cluster.cpp
	src/CluTSPDivision.cpp
	src/Graph.cpp
	src/Graph_Theory.cpp
	src/HoverLocation.cpp
	src/LKH_TSP_Solver.cpp
	src/Location.cpp
	src/Node.cpp
	src/PathTSP_MIP_PathPlanner.cpp
	src/Roots.cpp
	src/SolBasicMILP.cpp
	src/SolClusters.cpp
	src/SolHardMILP.cpp
	src/SolNearestNeighbor.cpp
	src/Solver.cpp
	src/Solution.cpp
	src/UAV_Stop.cpp
	src/Utilities.cpp
)


if(CXX)
	set(CMAKE_CXX_STANDARD 11)
	target_link_libraries(${CMAKE_PROJECT_NAME} optimized ${GUROBI_CXX_LIBRARY} debug ${GUROBI_CXX_DEBUG_LIBRARY})
endif()

target_link_libraries(${CMAKE_PROJECT_NAME} ${GUROBI_LIBRARY})

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
	include(FeatureSummary)
	feature_summary(WHAT ALL)
endif()
