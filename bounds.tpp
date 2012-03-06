#pragma once

#include "matrix_types.h"
#include "vector_utils.tpp"

namespace indoor_context {
	// Represents an axis-aligned bounding box
	template <typename T=double>
	class Bounds2D {
		T left_, right_, top_, bottom_;
	public:
		typedef Eigen::Matrix<T,2,1> Vec2T;
		typedef Eigen::Matrix<T,3,1> Vec3T;

		// Constructors
		Bounds2D() : left_(0), right_(0), top_(0), bottom_(0) { }
		Bounds2D(const T& left, const T& right, const T& top, const T& bottom)
			: left_(left), right_(right), top_(top), bottom_(bottom) { }

		// Accessors
		inline double left() const { return left_; }
		inline double right() const { return right_; }
		inline double top() const { return top_; }
		inline double bottom() const { return bottom_; }
		inline double width() const { return right_ - left_; }
		inline double height() const { return bottom_ - top_; }

		// Mutators
		inline void set_left(double v) { left_ = v; }
		inline void set_right(double v) { right_ = v; }
		inline void set_top(double v) { top_ = v; }
		inline void set_bottom(double v) { bottom_ = v; }

		// Get the coordinate of the corners
		Vec2T tl() const { return MakeVector<T>(left_, top_); }
		Vec2T tr() const { return MakeVector<T>(right_, top_); }
		Vec2T bl() const { return MakeVector<T>(left_, bottom_); }
		Vec2T br() const { return MakeVector<T>(right_, bottom_); }
		Vec2T center() const {
			return MakeVector<T>((left_+right_)/2, (top_+bottom_)/2);
		}

		// Get the coordinate of the corners in homogeneous coordinates
		Vec3T htl() const { return MakeVector<T>(left_, top_, 1); }
		Vec3T htr() const { return MakeVector<T>(right_, top_, 1); }
		Vec3T hbl() const { return MakeVector<T>(left_, bottom_, 1); }
		Vec3T hbr() const { return MakeVector<T>(right_, bottom_, 1); }

		// Get homogeneous line equations for the boundaries. Interior of
		// this region is on the positive side of these lines.
		Vec3T left_eqn() const { return MakeVector<T>(1,0,-left_); }
		Vec3T right_eqn() const { return MakeVector<T>(-1,0,right_); }
		Vec3T top_eqn() const { return MakeVector<T>(0,1,-top_); }
		Vec3T bottom_eqn() const { return MakeVector<T>(0,-1,bottom_); }

		// Construct bounds from any two opposing corners, in any order
		static Bounds2D<T> FromCorners(const Vec2& a, const Vec2& b) {
			return Bounds2D(std::min(a[0], b[0]), std::max(a[0], b[0]),
											std::min(a[1], b[1]), std::max(a[1], b[1]));
		}

		// Construct bounds from a size. Top-left corner is assumed to be at the origin.
		static Bounds2D<T> FromSize(const Vec2I& sz) {
			return Bounds2D(0, sz[0], 0, sz[1]);
		}

		// Construct bounds from a size. Top-left corner is assumed to be at the origin.
		static Bounds2D<T> FromTightSize(const Vec2I& sz) {
			return Bounds2D(0, sz[0]-1, 0, sz[1]-1);
		}
	};

	// Stream output operator for Bounds2D
	template <typename T>
	std::ostream& operator<<(std::ostream& s, const Bounds2D<T>& b) {
		s << "{left= " << b.left() << ", right=" << b.right()
			<< ", top=" << b.top() << ", bottom=" << b.bottom() << "}";
	}
}
