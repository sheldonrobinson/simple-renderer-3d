#include <iostream>

#include "common_types.h"

#include "numeric_utils.tpp"

namespace indoor_context {
	// Get a series of pairs [x0, x1] corresponding to horizontal strips in the image
	// that are within the bounds of the given polygon. The first scanline corresponds to y0
	// and then successive scanlines corresponding to successive rows.
	// Return the number of pixels within the polygon boundaries.
	int ComputeFillScanlines(const vector<Vec3>& poly,
													 const Vec2I& imsize,
													 int& y0,
													 vector<pair<int, int> >& scanlines) {
		const int n = poly.size();
		if (n < 3) return 0;

		int imin, imax;
		int ymin = numeric_limits<int>::max();
		int ymax = numeric_limits<int>::min();
		double ms[n], cs[n], ys[n];  // slopes, intercepts, y-coords
		bool ishoriz[n];
		for (int i = 0; i < n; i++) {
			const Vec3& u = poly[i];
			const Vec3& v = poly[(i+1)%n];
			if (isnan(u)) {
				cerr << "Warning: input vertex "<<i<<" has NaN coordinates in FillPolygon";
			}

			ys[i] = u[1] / u[2];

			// check if two vertices are coincident
			if (norm(project(u)-project(v)) < 1e-9) {
				ishoriz[i] = true;
			} else {
				Vec3 line = u ^ v;
				ishoriz[i] = abs(line[0]) < 1e-6*abs(line[1]);
				if (!ishoriz[i]) {
					ms[i] = -line[1]/line[0];
					cs[i] = -line[2]/line[0];
				}

				if (Ceili(ys[i]) < ymin) {
					ymin = Ceili(ys[i]);
					imin = i;
				}
				if (Floori(ys[i]) > ymax) {
					ymax = Floori(ys[i]);
					imax = i;
				}
			}
		}

		int count = 0;
		int left = (imin+n-1)%n;  // ring decrement
		int right = imin;

		bool encountered_nan = false;
		y0 = max(ymin, 0);
		int yb = min(ymax, imsize[1]);
		for (int y = y0; y < yb; y++) {
			while ((ys[left]<y || ishoriz[left]) && left != right) {
				left = (left+n-1)%n;  // ring decrement
			}
			while ((ys[(right+1)%n]<y || ishoriz[right]) && left != right) {
				right = (right+1)%n;  // ring increment
			}

			double xxa = ms[left]*y + cs[left];
			double xxb = ms[right]*y + cs[right];
			if (isnan(xxa) || isnan(xxb)) {
				// ignore and move to next row
				encountered_nan = true;
				continue;
			}

			int xa = Clamp(xxa, 0, imsize[0]-1);
			int xb = Clamp(xxb, 0, imsize[0]-1);
			count += abs(xa-xb);
			scanlines.push_back(make_pair(min(xa,xb), max(xa,xb)));
		}

		if (encountered_nan) {
			cerr << "Warning: NaN coordinates were computed during FillPolygon";
		}
	}
}  // namespace indoor_context
