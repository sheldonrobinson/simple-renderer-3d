#pragma once

#include "matrix_types.h"

namespace indoor_context {
	namespace { using Eigen::Dynamic; }

	// Divide all elements by the last and reduce size by 1
	template <typename T, int N>
	Eigen::Matrix<T,N-1,1> Project(const Eigen::Matrix<T,N,1>& v) {
		return v.template head<N-1>() / v[N-1];
	}

	// Append a "1" to the end of a vector
	template <typename T, int N>
	Eigen::Matrix<T,N+1,1> Unproject(const Eigen::Matrix<T,N,1>& v) {
		return Concatenate(v, 1.);
	}

	// Create a 2-vector
	template <typename T>
	Eigen::Matrix<T,2,1> MakeVector(const T& x1, const T& x2) {
		Eigen::Matrix<T,2,1> v;
		v << x1,x2;
		return v;
	}

	// Create a 3-vector
	template <typename T>
	Eigen::Matrix<T,3,1> MakeVector(const T& x1, const T& x2, const T& x3) {
		Eigen::Matrix<T,3,1> v;
		v << x1,x2,x3;
		return v;
	}

	// Create a 4-vector
	template <typename T>
	Eigen::Matrix<T,4,1> MakeVector(const T& x1, const T& x2,
																	const T& x3, const T& x4) {
		Eigen::Matrix<T,4,1> v;
		v << x1,x2,x3,x4;
		return v;
	}

	// Concatenateenate two vectors
	template <typename T>
	Eigen::Matrix<T,Dynamic,1> Concatenate(const Eigen::Matrix<T,Dynamic,1>& u,
																			 const Eigen::Matrix<T,Dynamic,1>& v) {
		Eigen::Matrix<T,Dynamic,1> r(u.size()+v.size());
		r.slice(0, u.size()) = u;
		r.slice(u.size(), v.size) = v;
		return r;
	}

	// Concatenateenate two vectors
	template <typename T, int N>
	Eigen::Matrix<T,Dynamic,1> Concatenate(const Eigen::Matrix<T,N,1>& u,
																				 const Eigen::Matrix<T,Dynamic,1>& v) {
		Eigen::Matrix<T,Dynamic,1> r(N+v.size());
		r.slice(0, N) = u;
		r.slice(N, v.size()) = v;
		return r;
	}

	// Concatenateenate two vectors
	template <typename T, int N>
	Eigen::Matrix<T,Dynamic,1> Concatenate(const Eigen::Matrix<T,Dynamic,1>& u,
																				 const Eigen::Matrix<T,N,1>& v) {
		Eigen::Matrix<T,Dynamic,1> r(u.size()+N);
		r.slice(0, u.size()) = u;
		r.slice(u.size(), N) = v;
		return r;
	}

	// Concatenateenate two vectors
	template <typename T, int M, int N>
	Eigen::Matrix<T,M+N,1> Concatenate(const Eigen::Matrix<T,M,1>& u,
																	 const Eigen::Matrix<T,N,1>& v) {
		Eigen::Matrix<T,M+N,1> r;
		r.template head<M>() = u;
		r.template tail<N>() = v;
		return r;
	}

	// Append a scalar to the end of a vector
	template <typename U, typename T, int N>
	Eigen::Matrix<T,N+1,1> Concatenate(const Eigen::Matrix<T,N,1>& v,
																		 const U& x) {
		Eigen::Matrix<T,N+1,1> r;
		r.template head<N>() = v;
		r[N] = x;
		return r;
	}

	// Append a scalar to the beginning of a vector
	template <typename U, typename T, int N>
	Eigen::Matrix<T,N+1,1> Concatenate(const U& x,
																		 const Eigen::Matrix<T,N,1>& v) {
		Eigen::Matrix<T,N+1,1> r;
		r.template tail<N>() = v;
		r[0] = x;
		return r;
	}

	// Divide the elements of a vector by the last element, as in
	// unproject(project(v)).
	template <typename T>
	inline Eigen::Matrix<T,3,1> AtRetina(const Eigen::Matrix<T,3,1>& x) {
		return MakeVector(x[0]/x[2], x[1]/x[2], 1.);
	}

	template <typename T>
	inline Eigen::Matrix<T,4,1> AtRetina(const Eigen::Matrix<T,4,1>& x) {
		return MakeVector(x[0]/x[3], x[1]/x[3], x[2]/x[3], 1.);
	}
}  // namespace indoor_context
