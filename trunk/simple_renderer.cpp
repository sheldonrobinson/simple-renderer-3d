#include "simple_renderer.h"

#include "common_types.h"
#include "clipping.h"
#include "depth_equation.h"
#include "fill_polygon.h"

#include "vector_utils.tpp"

namespace indoor_context {
	using namespace toon;

	static const double kExtent = 1e+3;  // extent of horizontal surfaces for RenderHorizSurface
	static const double kClampDepth = 1e+6;

	SimpleRenderer::SimpleRenderer() : viewport_(makeVector(0,0)) {
	}

	SimpleRenderer::SimpleRenderer(const toon::Matrix<3,4>& camera, Vec2I viewport) {
		Configure(camera, viewport);
	}

	void SimpleRenderer::Configure(const toon::Matrix<3,4>& camera, Vec2I viewport) {
		viewport_ = viewport;
		camera_ = camera;
		framebuffer_.Resize(viewport[1], viewport[0]);
		depthbuffer_.Resize(viewport[1], viewport[0]);
		Clear(0);
	}

	bool SimpleRenderer::Render(const Vec2& p, const Vec2& q, const Vec2& r, int label) {
		return Render(unproject(p), unproject(q), unproject(r), label);
	}

	bool SimpleRenderer::Render(const Vec3& p, const Vec3& q, const Vec3& r, int label) {
		if (viewport_[0] < 0 || viewport_[1] < 0) {
			cerr << "You must call SimpleRenderer::Configure() before Render()";
			return false;
		}

		// Do 3D clipping
		vector<Vec3> vs;
		vs.push_back(p);
		vs.push_back(q);
		vs.push_back(r);
		vector<Vec3> clipped;
		ClipToFrustrum(vs, camera_, viewport_, clipped);

		// Project into the camera
		vector<Vec3> projected;
		for (int i = 0; i < clipped.size(); i++) {
			projected.push_back(camera_ * unproject(clipped[i]));
		}

		// Compute the triangle scanlines
		int y0;
		vector<pair<int, int> > scanlines;
		ComputeFillScanlines(projected, viewport_, y0, scanlines);

		// Set up the depth equation
		Vec3 nrm = (p-q)^(p-r);
		Vec4 plane = Concatenate(nrm, -nrm*p);
		Vec3 depth_eqn = PlaneToDepthEqn(camera_, plane);

		// Do the rendering
		bool affected = false;
		for (int i = 0; i < scanlines.size(); i++) {
			// Pre-compute the first bit of the depth equation
			double depth_base = depth_eqn * makeVector(0, y0+i, 1);
			double depth_coef = depth_eqn[0];

			// Fill the row
			double* depth_row = depthbuffer_[y0+i];
			int* label_row = framebuffer_[y0+i];
			for (int x = scanlines[i].first; x <= scanlines[i].second; x++) {
				double depth = 1. / (depth_base + depth_coef*x);  // see PlaneToDepthEqn in geom_utils.h
				if (depth < 0) {
					// This can happen when a wall is almost exactly oblique to
					// the camera. In such a situation almost any ray intersection
					// will fail, though the scan line will include at least one
					// pixel so that pixel will generate a bogus depth. It should
					// be safe to ignore it as the surface behind this one will
					// pick up the depth.
					if (x != scanlines[i].first &&
							x != scanlines[i].second &&
							i != 0 &&
							i != scanlines.size()) {
						cerr << "Warning: negative depth="<<depth
								 << " at ("<<x<<","<<(y0+i)<<"),"
								 << " which is NOT on the boundary of a texel.";
					}
				} else if (depth < depth_row[x]) {
					depth_row[x] = depth;
					label_row[x] = label;
					affected = true;
				}
			}
		}

		return affected;
	}

	bool SimpleRenderer::RenderInfinitePlane(double z0, int label) {
		Vec4 plane = makeVector(0, 0, 1, -z0);
		Vec3 depth_eqn = PlaneToDepthEqn(camera_, plane);
		for (int y = 0; y < viewport_[1]; y++) {
			int* framerow = framebuffer_[y];
			double* depthrow = depthbuffer_[y];
			for (int x = 0; x < viewport_[0]; x++) {
				double depth = 1. / (depth_eqn * makeVector(x,y,1.));   // see PlaneToDepthEqn in geom_utils.h
				if (depth > 0) {
					framerow[x] = label;
					depthrow[x] = min(depth, kClampDepth);
				}
			}
		}
	}

	void SimpleRenderer::Clear(int bg) {
		framebuffer_.Fill(bg);
		depthbuffer_.Fill(INFINITY);
	}

	int SimpleRenderer::SmoothInfiniteDepths() {
		int n = 0;
		double maxdepth = 0;
		for (int y = 0; y < depthbuffer_.Rows(); y++) {
			double* row = depthbuffer_[y];
			for (int x = 0; x < depthbuffer_.Cols(); x++) {
				if (!isfinite(row[x]) || row[x] > kClampDepth) {
					// Pick a neighbour to replace with
					bool done = false;
					for (int dy = -1; !done && dy <= 1; dy+=2) {
						for (int dx = -1; !done && dx <= 1; dx+=2) {
							if (x+dx >= 0 && x+dx < depthbuffer_.Cols() &&
									y+dy >= 0 && y+dy < depthbuffer_.Rows()) {
								double v = depthbuffer_[y+dy][x+dx];
								if (isfinite(v) && v < kClampDepth) {
									row[x] = depthbuffer_[y+dy][x+dx];
									done = true;
								}
							}
						}
					}
					if (!done) {
						cerr << "A value in the depth buffer was infinite "
								 << "and ALL its neighbours were too. "
								 << "Please implement a better algorithm "
								 << "(e.g. walk towards image centre until valid value)";
					}
					n++;
				}
			}
		}
		if (!isfinite(depthbuffer_.MaxValue())) {
			cerr << "The algorithm in SmoothInfiniteDepths() has a bug!";
		}
		return n;
	}

}
