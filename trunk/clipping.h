#pragma once

#include <vector>
#include "matrix_types.h"

namespace indoor_context {
	// Get the line through two points, in Plucker coordinates
	PluckerLine LineThrough(const Vec3& x, const Vec3& y);

	// Get the line through two points, in Plucker coordinates
	PluckerLine LineThrough(const Vec4& x, const Vec4& y);

	// Get the intersection between a plane and a line (in Plucker coordinates)
	Vec4 PlaneLineIsct(const PluckerLine& m, const Vec4& w);

	// Clip a polygon to the positive side of a plane
	int ClipAgainstPlane(const std::vector<Vec3>& poly,
											 const Vec4& plane,
											 std::vector<Vec3>& out);

	// Clip a polygon against a frustrum
	int ClipToFrustrum(const std::vector<Vec3>& poly,
										 const LinearCamera& cam,
										 const Vec2I& viewport,
										 std::vector<Vec3>& out);
}
