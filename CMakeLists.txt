project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(COMMON_SOURCES
            src/behavior.cpp
            src/map.cpp
            src/path_planner.cpp
            src/point.cpp
            src/road.cpp
            src/spline.cpp
            src/straight_line_strategy.cpp
            src/walkthrough_strategy.cpp
            src/trajectory.cpp
            src/utils.cpp
            src/vehicle.cpp)

set(MAIN_SOURCE src/main.cpp)

set(POINT_SRC
            src/point.cpp
            src/spline.cpp
            src/trajectory.cpp
            src/utils.cpp
            src/map.cpp)

set(TEST_STRATEGY_SRC src/tests/strategy.cpp)
set(TEST_POINT_SRC src/tests/point.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${MAIN_SOURCE} ${COMMON_SOURCES})
add_executable(strategy_test ${TEST_STRATEGY_SRC} ${COMMON_SOURCES})
add_executable(point_test ${TEST_POINT_SRC} ${POINT_SRC})

target_link_libraries(path_planning z ssl uv uWS python2.7)
target_link_libraries(point_test z ssl uv uWS python2.7)
