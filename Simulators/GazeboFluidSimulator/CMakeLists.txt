set(SIMULATION_LINK_LIBRARIES SPlisHSPlasH Utilities )

#eigen
find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

############################################################
# NeighborhoodSearch
############################################################
include_directories(${NEIGHBORHOOD_SEARCH_INCLUDE_DIR})
set(SIMULATION_DEPENDENCIES ${SIMULATION_DEPENDENCIES} Ext_NeighborhoodSearch)
set(SIMULATION_LINK_LIBRARIES ${SIMULATION_LINK_LIBRARIES} ${NEIGHBORHOOD_SEARCH_LIBRARIES})

############################################################
# DiscreGrid
############################################################
include_directories(${PROJECT_PATH}/extern/install/Discregrid/include)
set(SIMULATION_DEPENDENCIES ${SIMULATION_DEPENDENCIES} Ext_Discregrid)
set(SIMULATION_LINK_LIBRARIES ${SIMULATION_LINK_LIBRARIES}
	optimized Discregrid
	debug Discregrid_d)
link_directories(${PROJECT_PATH}/extern/install/Discregrid/lib)

############################################################
# GenericParameters
############################################################
include_directories(${PROJECT_PATH}/extern/install/GenericParameters/include)
set(SIMULATION_DEPENDENCIES ${SIMULATION_DEPENDENCIES} Ext_GenericParameters)


#set(SIMULATION_LINK_LIBRARIES ${SIMULATION_LINK_LIBRARIES} 
#	optimized Utilities 
#	debug Utilities_d)
#link_directories(${PROJECT_PATH}/lib)


add_library(FluidSimulator SHARED
	FluidSimulator.h
	FluidSimulator.cpp
	GazeboSceneLoader.cpp
	GazeboSceneLoader.h
	GazeboSimulatorBase.cpp
	GazeboSimulatorBase.h
)

if(DL_OUTPUT)
	add_definitions(-DDL_OUTPUT)
endif()

set_property(TARGET FluidSimulator PROPERTY POSITION_INDEPENDENT_CODE ON)

add_definitions(-DTW_NO_LIB_PRAGMA -DTW_STATIC)
#include_directories(${PROJECT_PATH}/SPlisHSPlasH)
set_target_properties(FluidSimulator PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(FluidSimulator PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(FluidSimulator PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
add_dependencies(FluidSimulator ${SIMULATION_DEPENDENCIES})
message("FluidSimulator SIMULATION_LINK_LIBRARIES ${SIMULATION_LINK_LIBRARIES} ")
target_link_libraries(FluidSimulator ${SIMULATION_LINK_LIBRARIES} ${GAZEBO_LIBRARIES} ${IGNITION-MATH_LIBRARIES})
#target_link_libraries(FluidSimulator Utilities_d)

add_library(FluidVisPlugin SHARED FluidVisPlugin.cc)
target_link_libraries(FluidVisPlugin gazebo_msgs ${OGRE_LIBRARIES} gazebo_gui)
#set_target_properties(FluidVisPlugin PROPERTIES INSTALL_RPATH ${GAZEBO_PLUGIN_INSTALL_DIR})

set_target_properties(FluidSimulator PROPERTIES FOLDER "Simulators")

set(GAZEBO_PLUGIN_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/lib/gazebo-${GAZEBO_MAJOR_VERSION}/plugins
)
install(DIRECTORY ${PROJECT_PATH}/lib/ DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR} FILES_MATCHING PATTERN "*.so")