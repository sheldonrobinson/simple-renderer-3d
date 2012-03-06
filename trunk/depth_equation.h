#pragma once

#include "common_types.h"

namespace indoor_context {
	// Find an equation relating the (x,y) coordinates in an image to
	// the depth of a plane. The depth at pixel (x,y) is
	// 1./(eqn*makeVector(x,y,1)), where eqn is the return value from
	// this function
	Vec3 PlaneToDepthEqn(const toon::Matrix<3,4>& camera, const Vec4& plane);

	// Evaluate a depth equation as returned from PlaneToDepthEquation
	// at a particular image location.
	double EvaluateDepthEqn(const Vec3& depth_eqn, const Vec2& p);

	// Get the depth of a plane at a particular pixel
	double GetPlaneDepth(const Vec3& pixel,
											 const toon::Matrix<3,4>& camera,
											 const Vec4& plane);
}
