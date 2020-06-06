file(GLOB QDVO_BENCHMARKS 
"${PROJECT_SOURCE_DIR}/bench/*.cpp"
)

find_package (Boost REQUIRED COMPONENTS filesystem)

add_executable(benchmark_qdvo ${QDVO_BENCHMARKS})

target_link_libraries(benchmark_qdvo QDVO pthread benchmark Boost::filesystem)
