file(GLOB QDVO_TEST_FILES
    "${PROJECT_SOURCE_DIR}/test/*.h"
    "${PROJECT_SOURCE_DIR}/test/*.cpp"
)

enable_testing()
add_executable(test_qdvo ${QDVO_TEST_FILES})
target_link_libraries(test_qdvo ${GTEST_LIBRARIES} pthread QDVO ${OpenCV_LIBS} ${sophus_LIBRARIES} ${Boost_LIBRARIES} ${GTSAM_LIBRARIES})
gtest_discover_tests(test_qdvo)
#add_test(NAME qdvoTests COMMAND runTests)
