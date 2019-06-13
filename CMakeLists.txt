cmake_minimum_required(VERSION 3.5)
project (QDVO VERSION 0.0.1 LANGUAGES CXX)

#set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(CMAKE_CXX_FLAGS -Ofast)
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")

find_package(Ceres REQUIRED)
find_package (OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (Sophus REQUIRED)
find_package(GTest REQUIRED)
find_package(Boost REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${SOPHUS_INCLUDE_DIRS})

include_directories(
    ${PROJECT_SOURCE_DIR}/include/
    ${PROJECT_SOURCE_DIR}/include/CameraModels/
    ${PROJECT_SOURCE_DIR}/include/ErrorTerms/
    ${PROJECT_SOURCE_DIR}/include/FeatureDetection/
    ${PROJECT_SOURCE_DIR}/include/DataStructures/
    ${PROJECT_SOURCE_DIR}/include/FeatureMatching/
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
target_link_libraries(QDVO ${OpenCV_LIBS} ${Eigen_LIBRARIES} ${sophus_LIBRARIES} ${Ceres_LIBRARIES} ${Boost_LIBRARIES})


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

if(NOT DEFINED LIB_INSTALL_DIR)
        set(LIB_INSTALL_DIR lib)
endif()
install(TARGETS QDVO EXPORT QDVOTargets DESTINATION ${LIB_INSTALL_DIR})

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


