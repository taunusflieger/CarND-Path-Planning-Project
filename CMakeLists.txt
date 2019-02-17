project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall -fpermissive -Wno-unused-parameter -Werror")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -ggdb3 -DNDEBUG")
set(CMAKE_CXX_FLAGS "-std=c++17")


add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/config.cpp src/map.cpp src/vehicle.cpp src/jmt.cpp src/trajectory.cpp src/behavior.cpp src/prediction.cpp src/cost.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(src/include)
include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS python2.7)
