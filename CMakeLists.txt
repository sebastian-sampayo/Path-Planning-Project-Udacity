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
            src/spline_strategy.cpp
            src/straight_line_strategy.cpp
            src/walkthrough_strategy.cpp
            src/trajectory.cpp
            src/trajectory_cost.cpp
            src/utils.cpp
            src/vehicle.cpp)

set(MAIN_SOURCE src/main.cpp)

set(STRATEGY_SRC
            src/point.cpp
            src/spline.cpp
            src/trajectory.cpp
            src/utils.cpp
            src/spline_strategy.cpp
            src/straight_line_strategy.cpp
            src/walkthrough_strategy.cpp
            src/map.cpp)

set(POINT_SRC
            src/point.cpp
            src/spline.cpp
            src/trajectory.cpp
            src/utils.cpp
            src/map.cpp)

set(ROAD_SRC
            src/point.cpp
            src/road.cpp
            src/spline.cpp
            src/utils.cpp
            src/map.cpp
            src/vehicle.cpp)

set(TEST_PATH_PLANNER_SRC src/tests/path_planner.cpp)
set(TEST_STRATEGY_SRC src/tests/strategy.cpp)
set(TEST_POINT_SRC src/tests/point.cpp)
set(TEST_ROAD_SRC src/tests/road.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${MAIN_SOURCE} ${COMMON_SOURCES})
add_executable(path_planner_test ${TEST_PATH_PLANNER_SRC} ${COMMON_SOURCES})
add_executable(strategy_test ${TEST_STRATEGY_SRC} ${STRATEGY_SRC})
add_executable(point_test ${TEST_POINT_SRC} ${POINT_SRC})
add_executable(road_test ${TEST_ROAD_SRC} ${ROAD_SRC})

target_link_libraries(path_planning z ssl uv uWS python2.7)
target_link_libraries(point_test z ssl uv uWS python2.7)
target_link_libraries(road_test z ssl uv uWS python2.7)
target_link_libraries(strategy_test z ssl uv uWS python2.7)


# Add an option for debug version. Allows gdb debugging and defines macro DEBUG.
# If the option is not set, define a macro NDEBUG, so the asserts won't take effect.
option(DEFINE_DEBUG
  "Build the project using debugging code"
  OFF)
IF(DEFINE_DEBUG)
  message("Adding Debug flag...")
  add_definitions(-DDEBUG)
  set(CMAKE_BUILD_TYPE Debug)
  message("Build type is " ${CMAKE_BUILD_TYPE})
ELSE(DEFINE_DEBUG)
  message("Adding NotDebug flag...")
  add_definitions(-D NDEBUG)
ENDIF(DEFINE_DEBUG)
