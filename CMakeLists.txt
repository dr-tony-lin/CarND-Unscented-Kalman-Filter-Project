project(UnscentedKF)

# To check gcc support for AVX/SSE, run the following:
# gcc -mavx2 -dM -E - < /dev/null | egrep "SSE|AVX" | sort
# or, for avx-512:
# gcc -march=knl -dM -E - < /dev/null | egrep "SSE|AVX" | sort
cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

if((CMAKE_CXX_COMPILER_ID MATCHES "Clang") OR (CMAKE_CXX_COMPILER_ID MATCHES "GNU"))
  add_compile_options(-g -mavx2 -msse4.1 -mfma4)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
	add_compile_options(/arch:AVX2)
endif()

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/filter/utils.cpp src/filter/UKF.cpp src/filter/EKF.cpp)
include_directories(libs)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(kalman ${sources})

target_link_libraries(kalman z ssl uv uWS)
