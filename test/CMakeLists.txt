include(GoogleTest)
file(GLOB QDVO_TEST_FILES
    "${PROJECT_SOURCE_DIR}/test/*.h"
    "${PROJECT_SOURCE_DIR}/test/*.cpp"
)

find_package (Boost REQUIRED COMPONENTS filesystem)

enable_testing()
add_executable(test_qdvo ${QDVO_TEST_FILES})
target_link_libraries(test_qdvo gtest gtest_main ${OpenCV_LIBS} pthread QDVO sophus Boost::filesystem)
set(DISCOVERY_TIMEOUT 500)
gtest_discover_tests(test_qdvo)
