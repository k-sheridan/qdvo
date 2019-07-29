cmake_minimum_required(VERSION 3.5)
project (QDVO VERSION 0.0.1 LANGUAGES CXX)

#set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(CMAKE_CXX_FLAGS -O3)
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
include_directories(${Eigen_INCLUDE_DIRS})

include_directories(
    ${PROJECT_SOURCE_DIR}/include/
    ${PROJECT_SOURCE_DIR}/include/CameraModels/
    ${PROJECT_SOURCE_DIR}/include/ErrorTerms/
    ${PROJECT_SOURCE_DIR}/include/FeatureDetection/
    ${PROJECT_SOURCE_DIR}/include/DataStructures/
    ${PROJECT_SOURCE_DIR}/include/FeatureMatching/
    ${PROJECT_SOURCE_DIR}/include/Estimation/
    ${PROJECT_SOURCE_DIR}/include/Algorithm/
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


# add the library
add_library(QDVO ${QDVO_SOURCE_FILES} ${QDVO_HEADER_FILES})
target_link_libraries(QDVO ${OpenCV_LIBS} ${Eigen_LIBRARIES} ${sophus_LIBRARIES} ${Ceres_LIBRARIES} ${Boost_LIBRARIES})

set_property(TARGET QDVO PROPERTY CXX_STANDARD 17)

#Testing
option(PACKAGE_TESTS "Build the tests" ON)
if(PACKAGE_TESTS)
    enable_testing()
    #add_subdirectory(tests)
endif()


# unit tests
add_subdirectory(tests)




# Installation

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)


set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/QDVO)

install(TARGETS
            QDVO
        EXPORT
            QDVOTargets
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(EXPORT QDVOTargets
        FILE QDVOTargets.cmake
        NAMESPACE QDVO::
        DESTINATION ${INSTALL_CONFIGDIR}
)


install(FILES ${QDVO_HEADER_FILES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/QDVO)


# config file

#####################
# ConfigVersion file
##
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/QDVOConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
    ${CMAKE_CURRENT_LIST_DIR}/cmake/QDVOConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/QDVOConfig.cmake
    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

## Install all the helper files
install(
    FILES
      ${CMAKE_CURRENT_BINARY_DIR}/QDVOConfig.cmake
      ${CMAKE_CURRENT_BINARY_DIR}/QDVOConfigVersion.cmake
    DESTINATION ${INSTALL_CONFIGDIR}
)


target_include_directories(QDVO
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(QDVO::QDVO ALIAS QDVO)
