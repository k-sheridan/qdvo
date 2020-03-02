find_package(GTest)

if (GTest_FOUND)
file(GLOB QDVO_TEST_FILES
    "${PROJECT_SOURCE_DIR}/test/*.h"
    "${PROJECT_SOURCE_DIR}/test/*.cpp"
)

enable_testing()
add_executable(test_qdvo ${QDVO_TEST_FILES})
target_link_libraries(test_qdvo GTest::Main ${GTEST_LIBRARIES} ${OpenCV_LIBS} pthread QDVO ${sophus_LIBRARIES})
set(TEST_DISCOVERY_TIMEOUT 20)
gtest_discover_tests(test_qdvo)

endif()
