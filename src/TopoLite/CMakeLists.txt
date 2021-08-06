add_subdirectory(Stability)
add_subdirectory(IO)
add_subdirectory(Mesh)
add_subdirectory(Utility)

set(TpCoreFiles
        ${StabilityFiles}
        ${IOFiles}
        ${MeshFiles}
        ${UtilityFiles}
        PARENT_SCOPE)