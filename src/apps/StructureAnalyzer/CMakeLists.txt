cmake_minimum_required(VERSION 3.5)

project(StructureAnalyzer)

file(GLOB CRL_SOURCES #
     "${CMAKE_CURRENT_SOURCE_DIR}/*.h" #
     "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" #
)

list(
    APPEND
    CRL_TARGET_DEPENDENCIES #
    "kiki::rigid_block" #
    "kiki::render"
)

list(APPEND CRL_TARGET_INCLUDE_DIRS #
     PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
)

list(APPEND CRL_TARGET_LINK_LIBS #
    PUBLIC
    "kiki::rigid_block" #
    "kiki::render"
)

list(APPEND CRL_COMPILE_DEFINITIONS #
     PUBLIC "ROBOCRAFT_DATA_FOLDER=${ROBOCRAFT_DATA_FOLDER}" #
)

create_app(
    ${PROJECT_NAME}
    "${CRL_SOURCES}" #
    "${CRL_TARGET_DEPENDENCIES}" #
    "${CRL_TARGET_INCLUDE_DIRS}" #
    "${CRL_TARGET_LINK_LIBS}" #
    "${CRL_COMPILE_DEFINITIONS}"
)
