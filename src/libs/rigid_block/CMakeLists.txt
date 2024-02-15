cmake_minimum_required(VERSION 3.5)

project(rigid_block)

set(CRL_TARGET_NAME ${PROJECT_NAME})

file(
        GLOB
        CRL_SOURCES #
        "${CMAKE_CURRENT_SOURCE_DIR}/include/block/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/block/util/*.h" #
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/util/*.cpp"#
)

find_package(GUROBI REQUIRED)
find_package(Eigen3 REQUIRED)

# dependencies
list(
        APPEND
        CRL_TARGET_DEPENDENCIES #
        "Eigen3::Eigen"
)

# target include dirs
list(APPEND CRL_TARGET_INCLUDE_DIRS #
     PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
     PUBLIC "${GUROBI_INCLUDE_DIRS}")

# target link libs
list(
        APPEND
        CRL_TARGET_LINK_LIBS #
        PUBLIC
        "Eigen3::Eigen"
        PUBLIC
        "${GUROBI_LIBRARY}"
        PUBLIC
        "${GUROBI_CXX_LIBRARY}"
)

# compile definitions
list(
        APPEND
        CRL_COMPILE_DEFINITIONS #
        PUBLIC
        "CRL_DATA_FOLDER=${CRL_DATA_FOLDER}" #
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
