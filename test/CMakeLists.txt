set(CMAKE_MODULE_PATH   ${CMAKE_MODULE_PATH}
        ${CMAKE_CURRENT_SOURCE_DIR}/../ext/catch2/contrib)

message("${CMAKE_CURRENT_SOURCE_DIR}/../ext/catch2/contrib")


include(CTest)
include(ParseAndAddCatchTests)

add_subdirectory(Stability)
add_subdirectory(IO)
add_subdirectory(Mesh)
add_subdirectory(Utility)

set(testTopoFiles ${TestIO}
        ${TestUtility}
        ${TestMesh}
        ${TestStabilityFiles})

add_executable(testTopo ${testTopoFiles} Test_Main.cpp)

target_link_libraries(testTopo PUBLIC Catch2::Catch2)
target_link_libraries(testTopo PUBLIC TpCorelib)
ParseAndAddCatchTests(testTopo)

target_compile_definitions(testTopo PUBLIC -DTOPOCREATOR_DATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../result/")
