cmake_minimum_required(VERSION 3.5)

project(util)

set(CRL_TARGET_NAME ${PROJECT_NAME})

file(
        GLOB
        CRL_SOURCES #
        "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h" #
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" #
)
find_package(Eigen3 REQUIRED)

# basic dependencies
list(
        APPEND
        CRL_TARGET_DEPENDENCIES #
        "Eigen3::Eigen"
        "tinyxml2::tinyxml2"
)

# target include dirs
list(APPEND CRL_TARGET_INCLUDE_DIRS #
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

# target link libs
list(
        APPEND
        CRL_TARGET_LINK_LIBS #
        PUBLIC
        "Eigen3::Eigen"
        PUBLIC
        "tinyxml2::tinyxml2"
)

# compile definitions
list(APPEND
        CRL_COMPILE_DEFINITIONS #
        ""
)

# create target
create_library(
        ${CRL_TARGET_NAME}
        "${CRL_SOURCES}" #
        "${CRL_TARGET_DEPENDENCIES}" #
        "${CRL_TARGET_INCLUDE_DIRS}" #
        "${CRL_TARGET_LINK_LIBS}" #
        "${CRL_COMPILE_DEFINITIONS}"
)