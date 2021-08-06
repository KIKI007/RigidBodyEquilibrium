cmake_minimum_required(VERSION 3.1)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# eigen
set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/eigen)

# filesystem
set(FILESYSTEM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/filesystem)

# quickhull
find_package(QuickHull REQUIRED)

# clipper
find_package(Clipper REQUIRED)

# mosek
find_package(MOSEK)

#LIBIGL
set(LIBIGL_WITH_TRIANGLE ON CACHE BOOL "Use Triangle" FORCE)
set(LIBIGL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libigl/include)
list(APPEND CMAKE_MODULE_PATH "${LIBIGL_INCLUDE_DIR}/../cmake")
include(libigl)

# TBB
# Handled in cmake/TopoliteDependencies.cmake
if(WIN32)
    list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/tbb/cmake)
    find_package(TBB REQUIRED tbb_preview)
else()
    set(TBB_BUILD_STATIC            ON CACHE BOOL " " FORCE)
    set(TBB_BUILD_SHARED            OFF CACHE BOOL " " FORCE)
    set(TBB_BUILD_TBBMALLOC         ON  CACHE BOOL " " FORCE) # needed for CGAL's parallel mesher
    set(TBB_BUILD_TBBMALLOC_PROXY   OFF CACHE BOOL " " FORCE)
    set(TBB_BUILD_TESTS             OFF CACHE BOOL " " FORCE)
    add_subdirectory(${TOPOLITE_EXTERNAL}/tbb)
    set(TBB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tbb/include)
    set(TBB_IMPORTED_TARGETS tbb_static tbbmalloc_static)
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/catch2)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/catch2/contrib)

#JSON
set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(json)
set(JSON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/json/include)
#Set external inlude path/library/definitions


set(ext_include ${LIBIGL_INCLUDE_DIR}
                ${QUICKHULL_INCLUDE_DIR}
                ${CLIPPER_INCLUDE_DIR}
                ${TBB_INCLUDE_DIR}
                ${EIGEN3_INCLUDE_DIR}
                ${JSON_INCLUDE_DIR})

set(ext_lib ${TBB_IMPORTED_TARGETS}
            ${QUICKHULL_LIBRARIES}
            ${CLIPPER_LIBRARIES}
            igl::core
            igl::triangle
            nlohmann_json::nlohmann_json)

if(MOSEK_FOUND)
    set(ext_include ${ext_include} ${MOSEK_INCLUDE_DIRS})
    set(ext_lib ${ext_lib} ${MOSEK_LIBRARIES})
endif()

set_property(GLOBAL PROPERTY ext_include_property "${ext_include}")
set_property(GLOBAL PROPERTY ext_lib_property "${ext_lib}")
set_property(GLOBAL PROPERTY ext_defs_property "${ext_defs}")