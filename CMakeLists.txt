cmake_minimum_required(VERSION 3.14)
project(simu_analysis)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(Ceres CONFIG REQUIRED)
find_package(GeographicLib CONFIG REQUIRED)
find_package(Matplot++ CONFIG REQUIRED)
find_package(PythonLibs 3)

add_definitions(-DEIGEN_DONT_VECTORIZE)

    
add_executable (sensors_fusion_map
	main.cpp
	utils.cpp
	imu_process.cpp
	)


target_include_directories(${PROJECT_NAME}
	PUBLIC
	  ${PROJECT_SOURCE_DIR}/.*.hpp
	PRIVATE 
	  ${PYTHON_INCLUDE_DIRS}
	)

target_link_libraries(simu_analysis Ceres::ceres ${GeographicLib_LIBRARIES} Matplot++::cimg  Matplot++::matplot ${PYTHON_LIBRARIES})