cmake_minimum_required(VERSION 3.5)
project (QDVO VERSION 0.0.1 LANGUAGES CXX)

#set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#SET(CMAKE_CXX_FLAGS -O3)

find_package(Ceres REQUIRED)
find_package (OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (Sophus REQUIRED)
find_package(GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${SOPHUS_INCLUDE_DIRS})

include_directories(
    ${PROJECT_SOURCE_DIR}/include/
    ${PROJECT_SOURCE_DIR}/include/CameraModels/
    ${PROJECT_SOURCE_DIR}/include/ErrorTerms/
    ${PROJECT_SOURCE_DIR}/include/FeatureDetection/
    ${PROJECT_SOURCE_DIR}/include/DataStructures/
    )


file(GLOB QDVO_HEADER_FILES
    "${PROJECT_SOURCE_DIR}/include/*/*.h"
    "${PROJECT_SOURCE_DIR}/include/*/*.hpp"
    "${PROJECT_SOURCE_DIR}/include/*.h"
    "${PROJECT_SOURCE_DIR}/include/*.hpp"
)

file(GLOB QDVO_SOURCE_FILES
    "${PROJECT_SOURCE_DIR}/src/*/*.cpp"
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
)



include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# add the library
add_library(QDVO ${QDVO_SOURCE_FILES} ${QDVO_HEADER_FILES})
target_link_libraries(QDVO ${OpenCV_LIBS} ${Eigen_LIBRARIES} ${sophus_LIBRARIES} ${Ceres_LIBRARIES})


option(PACKAGE_TESTS "Build the tests" ON)
if(PACKAGE_TESTS)
    enable_testing()
    #add_subdirectory(tests)
endif()


# unit tests
add_subdirectory(tests)


# install
message("installing into ${CMAKE_INSTALL_PREFIX} with version ${CMAKE_PROJECT_VERSION}")

include(GNUInstallDirs)
set(QDVO_CMAKE_EXPORT_DIR ${CMAKE_INSTALL_DATADIR}/QDVO/cmake)
set_target_properties(QDVO PROPERTIES EXPORT_NAME QDVO)
install(TARGETS QDVO EXPORT QDVOTargets)
install(EXPORT QDVOTargets
  NAMESPACE QDVO::
  DESTINATION ${QDVO_CMAKE_EXPORT_DIR}
)
export(TARGETS QDVO NAMESPACE QDVO:: FILE QDVOTargets.cmake)
export(PACKAGE QDVO)



# Write version to file
write_basic_package_version_file (
  QDVOConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

# Install cmake targets
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/QDVOConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/QDVOConfigVersion.cmake
  DESTINATION ${QDVO_CMAKE_EXPORT_DIR}
)

# Install header files
install(
  FILES ${QDVO_HEADER_FILES}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/QDVO
  )


