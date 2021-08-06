include(FetchContent)

### Configuration
set(TOPOLITE_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")
set(TOPOLITE_EXTERNAL "${TOPOLITE_ROOT}/ext")

# Download and update 3rdparty libraries
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
list(REMOVE_DUPLICATES CMAKE_MODULE_PATH)
include(TopoliteDownloadExternal)

# Eigen
if(NOT TARGET Eigen3::Eigen)
    topolite_download_eigen()
endif()

# Catch2
if(NOT TARGET catch2::catch2)
    topolite_download_catch()
endif()

# libigl
if(NOT TARGET igl::core)
    topolite_download_libigl()
endif()

# TBB library
if(NOT TARGET tbb)
    if(WIN32)
        topolite_download_tbb_binary()
    else()
        topolite_download_tbb()
    endif()
endif()

#json
if(NOT TARGET nlohmann_json::nlohmann_json)
    topolite_download_json()
endif()