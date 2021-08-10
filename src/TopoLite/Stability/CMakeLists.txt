file(GLOB StabilityFiles
		${CMAKE_CURRENT_SOURCE_DIR}/ContactGraph/*.*
		${CMAKE_CURRENT_SOURCE_DIR}/Solver/Equilibrium/*.*
		${CMAKE_CURRENT_SOURCE_DIR}/Solver/Interlocking/*.*)
set(StabilityFiles ${StabilityFiles} PARENT_SCOPE)