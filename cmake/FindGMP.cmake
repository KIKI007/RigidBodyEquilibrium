 #Try to find the GNU Multiple Precision Arithmetic Library (GMP)
 # See http://gmplib.org/

 if (GMP_INCLUDES AND GMP_LIBRARIES)
     set(GMP_FIND_QUIETLY TRUE)
 endif (GMP_INCLUDES AND GMP_LIBRARIES)

 find_path(GMP_INCLUDES
         NAMES
         gmp.h
         PATHS
         $ENV{GMPDIR}
         ${INCLUDE_INSTALL_DIR}
         )

 find_library(GMPXX_LIB NAMES gmpxx PATHS $ENV{GMPDIR} ${LIB_INSTALL_DIR})
 find_library(GMP_LIB NAMES gmp PATHS $ENV{GMPDIR} ${LIB_INSTALL_DIR})
 set(GMP_LIBRARIES ${GMPXX_LIB} ${GMP_LIB})

 include(FindPackageHandleStandardArgs)
 find_package_handle_standard_args(GMP DEFAULT_MSG
         GMP_INCLUDES GMP_LIBRARIES)
 mark_as_advanced(GMP_INCLUDES GMP_LIBRARIES)