#include <SVD.h>

#include "common_types.h"

#include "vector_utils.tpp"

namespace indoor_context {
	using namespace toon;

	Vec3 PlaneToDepthEqn(const Matrix<3,4>& camera, const Vec4& plane) {
		Mat4 M;
		M.slice<0, 0, 3, 4> () = camera;
		M.slice<3, 0, 1, 4> () = plane.as_row();
		SVD<4> decomp(M);
		double du = 1. / (camera[2] * AtRetina(decomp.backsub(makeVector(1,0,1,0))));
		double dv = 1. / (camera[2] * AtRetina(decomp.backsub(makeVector(0,1,1,0))));
		double dw = 1. / (camera[2] * AtRetina(decomp.backsub(makeVector(0,0,1,0))));
		return makeVector(du-dw, dv-dw, dw);
	}

	double EvaluateDepthEqn(const Vec3& depth_eqn, const Vec2& p) {
		return 1. / (depth_eqn*unproject(p));
	}

	double GetPlaneDepth(const Vec3& p,
											 const Matrix<3,4>& camera,
											 const Vec4& plane) {
		Mat4 M;
		M.slice<0, 0, 3, 4> () = camera;
		M.slice<3, 0, 1, 4> () = plane.as_row();
		SVD<4> decomp(M);
		return camera[2] * AtRetina(decomp.backsub(Concatenate(p, 0.)));
	}
}
