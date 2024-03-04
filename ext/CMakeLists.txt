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
FetchContent_Declare(
        polyscope
        GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
        GIT_TAG        v2.1.0
)
FetchContent_MakeAvailable(polyscope)

#------------------------------------------------------------------------------
# json
FetchContent_Declare(
        json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.11.2
)
fetch(json)
add_subdirectory(${json_SOURCE_DIR} json)

#------------------------------------------------------------------------------
# json
FetchContent_Declare(
        tinyxml2
        GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
        GIT_TAG v10.0.0
)
fetch(tinyxml2)
add_subdirectory(${tinyxml2_SOURCE_DIR} tinyxml2)

#-----------------------------------------------------------------------------
# libccd
FetchContent_Declare(
        libccd
        GIT_REPOSITORY https://github.com/danfis/libccd.git
        GIT_TAG v2.1
)
fetch(libccd)
add_subdirectory(${libccd_SOURCE_DIR} libccd)

#------------------------------------------------------------------------------
# json
FetchContent_Declare(
        quickhull
        GIT_REPOSITORY git@github.com:KIKI007/quickhull.git
)
fetch(quickhull)
add_subdirectory(${quickhull_SOURCE_DIR} quickhull)
