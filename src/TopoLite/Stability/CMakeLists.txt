file(GLOB StabilityFiles
		${CMAKE_CURRENT_SOURCE_DIR}/ContactGraph/*.*
		${CMAKE_CURRENT_SOURCE_DIR}/Solver/Equilibrium/*.*
		${CMAKE_CURRENT_SOURCE_DIR}/Solver/Interlocking/*.*)
get_property(IPOPT_FOUND GLOBAL PROPERTY IPOPT_FOUND)
if(NOT IPOPT_FOUND)
	list(REMOVE_ITEM StabilityFiles "${CMAKE_CURRENT_SOURCE_DIR}/Solver/Interlocking/InterlockingSolver_Ipopt.cpp")
	list(REMOVE_ITEM StabilityFiles "${CMAKE_CURRENT_SOURCE_DIR}/Solver/Interlocking/InterlockingSolver_Ipopt.h")
endif() 
set(StabilityFiles ${StabilityFiles} PARENT_SCOPE)