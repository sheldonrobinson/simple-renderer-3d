PROJECT( SimpleRenderer3D )
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

###############################################################################
# CUSTOM VARIABLES
###############################################################################

# The variable controlling the 'light' version
#SET( LIGHT ON CACHE BOOL "Whether to build with minimal dependencies" )

###############################################################################
# INCLUDE LOCAL PATHS
###############################################################################
SET( CMAKE_MODULE_PATH 
	${CMAKE_SOURCE_DIR}/cmake_modules
	)

INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR} )

###############################################################################
# FIND DEPENDENCIES
###############################################################################
# Find Eigen
FIND_PACKAGE( Eigen3 REQUIRED )
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} )
MESSAGE( "Eigen: " ${EIGEN3_INCLUDE_DIR} )

###############################################################################
# BUILD LIBRARY
###############################################################################
ADD_LIBRARY( simplerenderer
	matrix_types.h
	numeric_utils.tpp
	vector_utils.tpp
	bounds.tpp

	clipping.h
	clipping.cpp

	depth_equation.h
	depth_equation.cpp

	fill_polygon.h
	fill_polygon.cpp

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

INSTALL(
#TODO
