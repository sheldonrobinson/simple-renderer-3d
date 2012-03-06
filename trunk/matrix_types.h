#pragma once

#define EIGEN_DEFAULT_TO_ROW_MAJOR
#include <Eigen/Core>

namespace indoor_context {
	typedef Eigen::Vector2d Vec2;
	typedef Eigen::Vector3d Vec3;
	typedef Eigen::Vector4d Vec4;

	typedef Eigen::Vector2i Vec2I;
	typedef Eigen::Vector3i Vec3I;
	typedef Eigen::Vector4i Vec4I;

	typedef Eigen::Matrix2d Mat2;
	typedef Eigen::Matrix3d Mat3;
	typedef Eigen::Matrix4d Mat4;

	typedef Eigen::VectorXi VecI;
	typedef Eigen::VectorXf VecF;
	typedef Eigen::VectorXd VecD;

	// A linear camera in 3D
	typedef Eigen::Matrix<double,3,4> LinearCamera;

	// Plucker coordinates are P=[V,X] where V is the direction of the
	// line and X is a point on the line.
	typedef Eigen::Matrix<double,6,1> PluckerLine;
}
