# FetchContent command is available with cmake >= 3.11
include(FetchContent)

# -----------------------------------------------------------------------------
# helper funtion to download a 'fetch content declare'
# -----------------------------------------------------------------------------
macro(fetch what)
    FetchContent_GetProperties("${what}")
    if(NOT ${${what}_POPULATED})
        message(STATUS "fetching ${what} ...")
        FetchContent_Populate(${what})
    endif()
    mark_as_advanced(${${what}_SOURCE_DIR})
endmacro()

# -----------------------------------------------------------------------------
# libigl
option(LIBIGL_GLFW             "Build target igl::glfw"             ON)
option(LIBIGL_IMGUI            "Build target igl::imgui"            ON)
option(LIBIGL_OPENGL           "Build target igl::opengl"           ON)
option(LIBIGL_COPYLEFT_CORE    "Build target igl_copyleft::core"    ON)
FetchContent_Declare(
        igl
        GIT_REPOSITORY https://github.com/libigl/libigl.git
        GIT_TAG        v2.4.0
)
FetchContent_MakeAvailable(igl)