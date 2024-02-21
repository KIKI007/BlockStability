cmake_minimum_required(VERSION 3.5)

project(robot)

set(CRL_TARGET_NAME ${PROJECT_NAME})

file(
        GLOB
        CRL_SOURCES #
        "${CMAKE_CURRENT_SOURCE_DIR}/include/robot/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" #
)

find_package(Eigen3 REQUIRED)

# dependencies
list(
        APPEND
        CRL_TARGET_DEPENDENCIES #
        "Eigen3::Eigen"
        "util"
)

# target include dirs
list(APPEND CRL_TARGET_INCLUDE_DIRS #
     PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

# target link libs
list(
        APPEND
        CRL_TARGET_LINK_LIBS #
        PUBLIC
        "Eigen3::Eigen"
        "util"
)

# compile definitions
list(
        APPEND
        CRL_COMPILE_DEFINITIONS #
        PUBLIC "ROBOCRAFT_DATA_FOLDER=${ROBOCRAFT_DATA_FOLDER}" #
)

# create target
create_library(
        ${CRL_TARGET_NAME}
        "${CRL_SOURCES}" #
        "${CRL_TARGET_DEPENDENCIES}" #
        "${CRL_TARGET_INCLUDE_DIRS}" #
        "${CRL_TARGET_LINK_LIBS}" #
        "${CRL_COMPILE_DEFINITIONS}" #
)
