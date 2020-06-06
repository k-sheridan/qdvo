cmake_minimum_required(VERSION 3.5)
project (QDVO VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT CMAKE_BUILD_TYPE MATCHES "Debug" AND NOT QDVO_DISABLE_OPTIMIZATIONS)
message("Building with O3 on.")
SET(CMAKE_CXX_FLAGS "-Ofast -march=native")
else() 
message("Building without O3.")
SET(CMAKE_CXX_FLAGS "-O0")
endif()

if (QDVO_ASAN)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer -fsanitize=address")
SET(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fno-omit-frame-pointer -fsanitize=address")
elseif(QDVO_UBSAN)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined")
SET(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=undefined")
elseif(QDVO_TSAN)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=thread")
SET(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=thread")
elseif(QDVO_MSAN)
SET(CMAKE_CXX_FLAGS "-O1 -fsanitize=memory -fno-omit-frame-pointer")
SET(CMAKE_LINKER_FLAGS "-O1 -fsanitize=memory -fno-omit-frame-pointer")
endif()


set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

find_package (OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)

include_directories(${benchmark_INCLUDE_DIRS})
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(
    ${PROJECT_SOURCE_DIR}/src/
    ${PROJECT_SOURCE_DIR}/src/CameraModels/
    ${PROJECT_SOURCE_DIR}/src/ErrorTerms/
    ${PROJECT_SOURCE_DIR}/src/FeatureDetection/
    ${PROJECT_SOURCE_DIR}/src/FeatureMatching/
    ${PROJECT_SOURCE_DIR}/src/Estimation/
    ${PROJECT_SOURCE_DIR}/src/ParallelAlgorithms/
    ${PROJECT_SOURCE_DIR}/src/Pipeline/
    ${PROJECT_SOURCE_DIR}/thirdparty/
    )

file(GLOB QDVO_HEADER_FILES
    "${PROJECT_SOURCE_DIR}/src/*/*.h"
    "${PROJECT_SOURCE_DIR}/src/*/*/*.h"
    "${PROJECT_SOURCE_DIR}/src/*/*.hpp"
    "${PROJECT_SOURCE_DIR}/src/*.h"
    "${PROJECT_SOURCE_DIR}/src/*.hpp"
)

file(GLOB QDVO_SOURCE_FILES
    "${PROJECT_SOURCE_DIR}/src/*/*.cpp"
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

# Add thirdparty libraries
add_subdirectory(thirdparty/googletest)
add_subdirectory(thirdparty/benchmark)
add_subdirectory(thirdparty/gflags)
add_subdirectory(thirdparty/spdlog)
add_subdirectory(thirdparty/Sophus)
add_subdirectory(thirdparty/yaml-cpp)

SET(CMAKE_C_COMPILER clang)
SET(CMAKE_CXX_COMPILER clang++)

# add the library
add_library(QDVO ${QDVO_SOURCE_FILES} ${QDVO_HEADER_FILES})
target_link_libraries(QDVO ${OpenCV_LIBS} sophus spdlog)

set_property(TARGET QDVO PROPERTY CXX_STANDARD 17)

# unit tests
add_subdirectory(test)

# benchmark
add_subdirectory(bench)

# visualization
add_subdirectory(visualizer)

# Installation
if(INSTALL_QDVO)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)


set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/QDVO)

install(TARGETS
            QDVO
	    spdlog
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
endif()

# debug cmake
#macro(print_all_variables)
 #   message(STATUS "print_all_variables------------------------------------------{")
 #   get_cmake_property(_variableNames VARIABLES)
 #   foreach (_variableName ${_variableNames})
 #       message(STATUS "${_variableName}=${${_variableName}}")
 #   endforeach()
#    message(STATUS "print_all_variables------------------------------------------}")
#endmacro()

#print_all_variables()
