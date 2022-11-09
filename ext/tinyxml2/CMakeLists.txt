project(tinyxml2)

add_library(tinyxml2 STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tinyxml2.cpp)
target_include_directories(tinyxml2 PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
)