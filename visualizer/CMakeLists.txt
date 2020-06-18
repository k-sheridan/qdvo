find_package(Pangolin)

if (Pangolin_FOUND)
# visualizer
add_library(qdvo_visualizer QDVOVisualizer.cpp)
target_link_libraries(qdvo_visualizer QDVO ${Pangolin_LIBRARIES} ${OpenCV_LIBS})

# execs
add_executable(runEurocDatasets eurocdatasetplayer.cpp)
target_link_libraries(runEurocDatasets qdvo_visualizer QDVO gflags ${OpenCV_LIBS} yaml-cpp) 
endif()
