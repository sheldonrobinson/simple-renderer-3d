#include "clipping.h"

#include "common_types.h"

#include "bounds.tpp"
#include "vector_utils.tpp"

namespace indoor_context {
	using namespace toon;

	static const double kZNear = 1e-6;
	static const double kZFar = 1e+6;

	// Note: Plucker coordinates are P=[V,X] where V is the direction of
	// the line and X is a point on the line.

	// Get the line through two points, in Plucker coordinates
	Vec6 LineThrough(const Vec3& x, const Vec3& y) {
		Vec6 m;
		m.slice<0,3>() = x-y;
		m.slice<3,3>() = x^y;
		return m;
	}

	// Get the line through two points, in Plucker coordinates
	Vec6 LineThrough(const Vec4& x, const Vec4& y) {
		Vec6 m;
		m.slice<0,3>() = y[3]*x.slice<0,3>() - x[3]*y.slice<0,3>();
		m.slice<3,3>() = x.slice<0,3>() ^ y.slice<0,3>();
		return m;
	}

	// Get the intersection between a plane and a line (in Plucker coordinates)
	Vec4 PlaneLineIsct(const Vec6& m, const Vec4& w) {
		Vec4 p;
		p.slice<0,3>() = (m.slice<3,3>()^w.slice<0,3>()) - w[3]*m.slice<0,3>();
		p[3] = w.slice<0,3>() * m.slice<0,3>();
		return p;
	}

	// Clip a polygon to the positive side of a plane
	int ClipAgainstPlane(const vector<Vec3>& poly,
											 const Vec4& plane,
											 vector<Vec3>& out) {
		int n = poly.size();
		bool prev = false;
		for (int i = 0; i < n; i++) {
			const Vec3& a = poly[i];
			const Vec3& b = poly[(i+1)%n];
			bool aa = a*plane.slice<0,3>() + plane[3] >= 0;
			bool bb = b*plane.slice<0,3>() + plane[3] >= 0;
			if (aa || bb) {
				Vec3 isct = aa && bb ? Zeros : project(PlaneLineIsct(LineThrough(a,b), plane));
				if (!aa) {
					out.push_back(isct);
				} else if (!prev) {
					out.push_back(a);
				}
				if (!bb) {
					out.push_back(isct);
				} else if (i+1 < n) {
					out.push_back(b);
				}
			}
			prev = bb;
		}
	}

	// Clip a polygon against a frustrum
	int ClipToFrustrum(const vector<Vec3>& poly,
										 const Matrix<3,4>& cam,
										 const Vec2I& viewport,
										 vector<Vec3>& out) {
		// Create the frustrum in camera coords
		Bounds2D<> vp = Bounds2D<>::FromSize(viewport);
		Vec4 frustrum[] = {
			makeVector(0.0, 0, 1, -kZNear),
			makeVector(0.0, 0, -1, kZFar),
			Concatenate(vp.left_eqn(), 0),
			Concatenate(vp.right_eqn(), 0),
			Concatenate(vp.top_eqn(), 0),
			Concatenate(vp.bottom_eqn(), 0)
		};

		// Construct the camera matrix
		Matrix<4> m = Identity;
		m.slice<0,0,3,4>() = cam;

		// Transfer planes _from camera to world_ using the _forwards_ camera matrix
		vector<Vec3> temp1, temp2;
		std::copy(poly.begin(), poly.end(), back_inserter(temp1));
		for (int i = 0; i < 6; i++) {
			ClipAgainstPlane(temp1, m.T()*frustrum[i], temp2);
			swap(temp1,temp2);
			temp2.clear();
		}
		std::copy(temp1.begin(), temp1.end(), back_inserter(out));
	}
}
