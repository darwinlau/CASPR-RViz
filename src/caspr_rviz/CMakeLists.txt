cmake_minimum_required(VERSION 2.8.3)
project(caspr_rviz)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/devel/lib)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g ")

SET(CMAKE_BUILD_TYPE Debug)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED COMPONENTS roscpp rosbag std_msgs geometry_msgs
	message_generation rviz tf tf_conversions
)

catkin_package(
	CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

include_directories(
		include/
				include/rviz
        ${catkin_INCLUDE_DIRS}
				${Boost_INCLUDE_DIRS}
)
#==============================================================================
# Application
#==============================================================================


#==============================================================================
# Executable
#==============================================================================
add_executable(caspr_rviz src/CASPR_RViz_Main.cpp src/Visualization.cpp)
target_link_libraries(caspr_rviz ${Boost_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS caspr_rviz
				RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
