###############################################################################
# BUILD EXAMPLES
###############################################################################
SET( EXAMPLES
	foo
	unittest
	)

FOREACH( EXAMPLE ${EXAMPLES} )
	ADD_EXECUTABLE( ${EXAMPLE} ${EXAMPLE}.cpp )
	TARGET_LINK_LIBRARIES( ${EXAMPLE}
		simplerenderer
		${EXTERNAL_LIBRARIES}
	)
ENDFOREACH( EXAMPLE )
