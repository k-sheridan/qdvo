find_package(GoogleBenchmark REQUIRED)

if (benchmark_FOUND)
file(GLOB QDVO_BENCHMARKS 
"${PROJECT_SOURCE_DIR}/bench/*.cpp"
)

add_executable(benchmark_qdvo ${QDVO_BENCHMARKS})

target_link_libraries(benchmark_qdvo QDVO pthread ${benchmark_LIBRARIES} ${OpenCV_LIBS} ${sophus_LIBRARIES} ${Boost_LIBRARIES} ${GTSAM_LIBRARIES})
endif()
