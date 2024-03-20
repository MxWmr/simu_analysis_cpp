cmake_minimum_required (VERSION 3.8)

project (simu_analysis)

# C++17 required
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Debug")

# Required packages
find_package(Eigen3 CONFIG REQUIRED)
find_package(Ceres CONFIG REQUIRED)

# Prevent Eigen Alignment issues
add_definitions(-DEIGEN_DONT_VECTORIZE)

add_executable(${PROJECT_NAME}
	main.cpp
	utils.cpp
	imu_process.cpp
	DataWriter.cpp
)

# target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen Ceres::ceres)

