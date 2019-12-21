find_package(Pangolin)
find_package(GFLAGS)

# visualizer
add_library(qdvo_visualizer QDVOVisualizer.cpp)
target_link_libraries(qdvo_visualizer QDVO ${Pangolin_LIBRARIES} ${OpenCV_LIBS})

# execs
add_executable(runEurocDatasets eurocdatasetplayer.cpp)
target_link_libraries(runEurocDatasets qdvo_visualizer QDVO ${OpenCV_LIBS} ${Eigen_LIBRARIES} ${Sophus_LIBRARIES} ${GFLAGS_LIBRARIES})
