cmake_minimum_required(VERSION 3.14)
project(simu_analysis)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(Ceres CONFIG REQUIRED)
find_package(GeographicLib CONFIG REQUIRED)
find_package(Matplot++ CONFIG REQUIRED)

# Required packages
find_package(Eigen3 CONFIG REQUIRED)

# Prevent Eigen Alignment issues
add_definitions(-DEIGEN_DONT_VECTORIZE)
    
add_executable (simu_analysis
	main.cpp
	utils.cpp
	imu_process.cpp
	DataWriter.cpp
	)




target_link_libraries(simu_analysis Ceres::ceres ${GeographicLib_LIBRARIES} Matplot++::cimg Matplot++::matplot Eigen3::Eigen)