#include "clipping.h"

#include <vector>

#include <Eigen/Geometry>

#include "matrix_types.h"

#include "bounds.tpp"
#include "vector_utils.tpp"
#include "numeric_utils.tpp"

namespace indoor_context {
	static const double kZNear = 1e-6;
	static const double kZFar = 1e+6;

	using std::vector;

	// Get the line through two points, in Plucker coordinates
	PluckerLine LineThrough(const Vec3& x, const Vec3& y) {
		PluckerLine m;
		m.head<3>() = x-y;
		m.tail<3>() = x.cross(y);
		return m;
	}

	// Get the line through two points, in Plucker coordinates
	PluckerLine LineThrough(const Vec4& x, const Vec4& y) {
		PluckerLine m;
		m.head<3>() = y[3]*x.head<3>() - x[3]*y.head<3>();
		m.tail<3>() = x.head<3>().cross( y.head<3>() );
		return m;
	}

	// Get the intersection between a plane and a line (in Plucker coordinates)
	Vec4 PlaneLineIsct(const PluckerLine& m, const Vec4& w) {
		Vec4 p;
		p.head<3>() = m.tail<3>().cross( w.head<3>() ) - w[3]*m.head<3>();
		p[3] = w.head<3>().dot( m.head<3>() );
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
			bool aa = a.dot(plane.head<3>()) + plane[3] >= 0;
			bool bb = b.dot(plane.head<3>()) + plane[3] >= 0;
			if (aa || bb) {
				Vec3 isct = aa && bb ? Vec3::Zero() : Project(PlaneLineIsct(LineThrough(a,b), plane));
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
										 const LinearCamera& cam,
										 const Vec2I& viewport,
										 vector<Vec3>& out) {
		// Create the frustrum in camera coords
		Bounds2D<> vp = Bounds2D<>::FromSize(viewport);
		Vec4 frustrum[] = {
			MakeVector<double>(0, 0, 1, -kZNear),
			MakeVector<double>(0, 0, -1, kZFar),
			Concatenate(vp.left_eqn(), 0),
			Concatenate(vp.right_eqn(), 0),
			Concatenate(vp.top_eqn(), 0),
			Concatenate(vp.bottom_eqn(), 0)
		};

		// Construct the camera matrix
		Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
		m.topRows<3>() = cam;

		// Transfer planes _from camera to world_ using the _forwards_ camera matrix
		vector<Vec3> temp1, temp2;
		std::copy(poly.begin(), poly.end(), back_inserter(temp1));
		for (int i = 0; i < 6; i++) {
			ClipAgainstPlane(temp1, m.transpose()*frustrum[i], temp2);
			swap(temp1,temp2);
			temp2.clear();
		}
		std::copy(temp1.begin(), temp1.end(), back_inserter(out));
	}
}
