#include "depth_equation.h"

#include <Eigen/SVD>

#include "matrix_types.h"
#include "vector_utils.tpp"

namespace indoor_context {
	Vec3 PlaneToDepthEqn(const LinearCamera& camera, const Vec4& plane) {
		Mat4 M;
		M.topRows<3>() = camera;
		M.row(3) = plane;
		Eigen::JacobiSVD<Mat4> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
		double du = 1. / camera.row(2).dot(AtRetina(svd.solve(MakeVector<double>(1,0,1,0)).eval()));
		double dv = 1. / camera.row(2).dot(AtRetina(svd.solve(MakeVector<double>(0,1,1,0)).eval()));
		double dw = 1. / camera.row(2).dot(AtRetina(svd.solve(MakeVector<double>(0,0,1,0)).eval()));
		return MakeVector(du-dw, dv-dw, dw);
	}

	double EvaluateDepthEqn(const Vec3& depth_eqn, const Vec2& p) {
		return 1. / (depth_eqn.dot(Unproject(p)));
	}

	double GetPlaneDepth(const Vec3& p,
											 const LinearCamera& camera,
											 const Vec4& plane) {
		Mat4 M;
		M.topRows<3>() = camera;
		M.bottomRows<1>() = plane;
		return camera.row(2).dot( M.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV)
															 .solve(Concatenate(p, 0.)) );
	}
}
