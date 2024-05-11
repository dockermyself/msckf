cmake_minimum_required (VERSION 3.5)

project (msckf-vio)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -fpermissive")
#Debug or Release
set(CMAKE_BUILD_TYPE "Release")
# add OpenCV library
find_package (OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

# add Eigen library
find_package (Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# compile msckf_vio library
include_directories (
    ${PROJECT_SOURCE_DIR}/msckf_vio
)

add_executable (
	msckf_vio
    ${PROJECT_SOURCE_DIR}/msckf_vio/main.cpp
	${PROJECT_SOURCE_DIR}/msckf_vio/core/image_processor.cpp
	${PROJECT_SOURCE_DIR}/msckf_vio/core/msckf_vio.cpp
	${PROJECT_SOURCE_DIR}/msckf_vio/core/msckf_sys.cpp
	${PROJECT_SOURCE_DIR}/msckf_vio/core/params.cpp)

target_link_libraries (msckf_vio ${OpenCV_LIBS} -pthread)

