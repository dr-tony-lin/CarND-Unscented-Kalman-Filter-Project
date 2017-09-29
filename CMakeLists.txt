project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/filter/utils.cpp src/filter/UKF.cpp src/filter/EKF.cpp)
set(sources2 src/catch_run_away_car.cpp src/filter/utils.cpp src/filter/UKF.cpp src/filter/EKF.cpp)
include_directories(libs)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(kalman ${sources})
add_executable(catch_run ${sources2})

target_link_libraries(kalman z ssl uv uWS)
target_link_libraries(catch_run z ssl uv uWS)