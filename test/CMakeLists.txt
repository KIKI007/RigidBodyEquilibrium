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

add_executable(testRBE Stability/Test_EquilibriumFriction_Mosek.cpp Test_Main.cpp)

target_link_libraries(testRBE PUBLIC Catch2::Catch2)
target_link_libraries(testRBE PUBLIC RBECorelib)
ParseAndAddCatchTests(testRBE)