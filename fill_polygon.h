#pragma once

#include "common_types.h"

namespace indoor_context {
	// Get a series of pairs [x0, x1] corresponding to horizontal strips in the image
	// that are within the bounds of the given polygon. The first scanline corresponds to y0
	// and then successive scanlines corresponding to successive rows.
	// Return the number of pixels within the polygon boundaries.
	int ComputeFillScanlines(const vector<Vec3>& poly,
													 const Vec2I& imsize,
													 int& y0,
													 vector<pair<int, int> >& scanlines);
}  // namespace indoor_context
