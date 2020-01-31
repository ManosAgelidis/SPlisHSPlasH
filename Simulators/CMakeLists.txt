include(${PROJECT_PATH}/Visualization/CMakeLists.txt)
add_definitions(-DPBD_DATA_PATH="../data")

subdirs( GazeboFluidSimulator )  #GazeboRosFluidSimulator/src DynamicBoundarySimulator StaticBoundarySimulator

