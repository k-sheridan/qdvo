set(QDVO_TEST_FILES
    ${PROJECT_SOURCE_DIR}/tests/MainTest.cpp
    ${PROJECT_SOURCE_DIR}/tests/TestQDVOHelpers.cpp
    ${PROJECT_SOURCE_DIR}/tests/TestCameraModel.cpp
)

enable_testing()
add_executable(runTests ${QDVO_TEST_FILES})
target_link_libraries(runTests ${GTEST_LIBRARIES} pthread)
add_test(NAME qdvoTests COMMAND runTests)
