#pragma once

#include "common_types.h"

#include "numeric_utils.tpp"

namespace indoor_context {
	// Get the line through two points, in Plucker coordinates
	Vec6 LineThrough(const Vec3& x, const Vec3& y);

	// Get the line through two points, in Plucker coordinates
	Vec6 LineThrough(const Vec4& x, const Vec4& y);

	// Get the intersection between a plane and a line (in Plucker coordinates)
	Vec4 PlaneLineIsct(const Vec6& m, const Vec4& w);

	// Clip a polygon to the positive side of a plane
	int ClipAgainstPlane(const vector<Vec3>& poly,
											 const Vec4& plane,
											 vector<Vec3>& out);

	// Clip a polygon against a frustrum
	int ClipToFrustrum(const vector<Vec3>& poly,
										 const toon::Matrix<3,4>& cam,
										 const Vec2I& viewport,
										 vector<Vec3>& out);
}
