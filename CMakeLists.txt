PROJECT( SimpleRenderer3D )
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

# Setup the path to find modules to prefer local modules
SET( CMAKE_MODULE_PATH 
	${CMAKE_SOURCE_DIR}/cmake_modules
	)

# Set up system-level search paths
LIST(APPEND CMAKE_LIBRARY_PATH
	/usr/lib/x86_64-linux-gnu/
	)

###############################################################################
# CUSTOM VARIABLES
###############################################################################

# The variable controlling the 'light' version
#SET( LIGHT ON CACHE BOOL "Whether to build with minimal dependencies" )

###############################################################################
# INCLUDE LOCAL PATHS
###############################################################################

INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR} )

###############################################################################
# FIND DEPENDENCIES
###############################################################################

# Find Eigen
FIND_PACKAGE( Eigen3 REQUIRED )
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} )
MESSAGE( "Eigen: " ${EIGEN3_INCLUDE_DIR} )

# Find libjpeg
#FIND_PACKAGE( JPEG REQUIRED )
#INCLUDE_DIRECTORIES( ${JPEG_INCLUDE_DIR} )
#LIST( APPEND EXTERNAL_LIBRARIES ${JPEG_LIBRARIES} )

# Find libpng
#FIND_PACKAGE( PNG REQUIRED )
#INCLUDE_DIRECTORIES( ${PNG_INCLUDE_DIR} )
#LIST( APPEND EXTERNAL_LIBRARIES ${PNG_LIBRARIES} )

# Find Boost
#SET( Boost_FIND_QUIETLY OFF )
#FIND_PACKAGE( Boost 1.44.0
#	COMPONENTS date_time filesystem iostreams thread system program_options python
#	REQUIRED )
#LIST(APPEND EXTERNAL_LIBRARIES 
#	${Boost_system_LIBRARY_RELEASE}
#	${Boost_date_time_LIBRARY}
#	${Boost_filesystem_LIBRARY}
#	${Boost_iostreams_LIBRARY}
#	${Boost_thread_LIBRARY}
#	${Boost_program_options_LIBRARY}
#)
#ADD_DEFINITIONS( "-DBOOST_FILESYSTEM_VERSION=3" )

# Find LAPACK
# Note that FindLAPACK.cmake seems only to support Fortran
FIND_LIBRARY( LAPACK_LIBRARY liblapack.so )
IF( NOT LAPACK_LIBRARY )
	MESSAGE( FATAL_ERROR "Could not find LAPACK" )
ENDIF()
INCLUDE_DIRECTORIES( ${LAPACK_INCLUDE_DIR} )
LIST( APPEND EXTERNAL_LIBRARIES ${LAPACK_LIBRARY} )

###############################################################################
# BUILD LIBRARY
###############################################################################
ADD_LIBRARY( simplerenderer
	bounds.tpp

	clipping.h
	clipping.cpp

	depth_equation.h
	depth_equation.cpp

	fill_polygon.h
	fill_polygon.cpp

	matrix_types.h
	numeric_utils.tpp
	vector_utils.tpp

	simple_renderer.h
	simple_renderer.cpp
)

TARGET_LINK_LIBRARIES( simplerenderer ${EXTERNAL_LIBRARIES} )

###############################################################################
# BUILD EXAMPLES
###############################################################################
ADD_SUBDIRECTORY(examples)

###############################################################################
# INSTALL
###############################################################################

#TODO
