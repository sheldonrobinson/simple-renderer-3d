#pragma once

#include "common_types.h"

namespace indoor_context {
	// Concatenateenate two vectors
	template <typename T>
	toon::Vector<toon::Dynamic,T> Concatenate(const toon::Vector<toon::Dynamic,T>& u,
																			 const toon::Vector<toon::Dynamic,T>& v) {
		toon::Vector<toon::Dynamic,T> r(u.size()+v.size());
		r.slice(0, u.size()) = u;
		r.slice(u.size(), v.size) = v;
		return r;
	}

	// Concatenateenate two vectors
	template <typename T, int N>
	toon::Vector<toon::Dynamic,T> Concatenate(const toon::Vector<N,T>& u,
																			 const toon::Vector<toon::Dynamic,T>& v) {
		toon::Vector<toon::Dynamic,T> r(N+v.size());
		r.slice(0, N) = u;
		r.slice(N, v.size()) = v;
		return r;
	}

	// Concatenateenate two vectors
	template <typename T, int N>
	toon::Vector<toon::Dynamic,T> Concatenate(const toon::Vector<toon::Dynamic,T>& u,
																			 const toon::Vector<N,T>& v) {
		toon::Vector<toon::Dynamic,T> r(u.size()+N);
		r.slice(0, u.size()) = u;
		r.slice(u.size(), N) = v;
		return r;
	}

	// Concatenateenate two vectors
	template <typename T, int M, int N>
	toon::Vector<M+N,T> Concatenate(const toon::Vector<M,T>& u, const toon::Vector<N,T>& v) {
		toon::Vector<M+N,T> r;
		r.template slice<0,M>() = u;
		r.template slice<M,N>() = v;
		return r;
	}

	// Append a scalar to the end of a vector
	template <typename U, typename T, int N>
	toon::Vector<N+1,T> Concatenate(const toon::Vector<N,T>& v, const U& x) {
		toon::Vector<N+1,T> r;
		r.template slice<0,N>() = v;
		r[N] = x;
		return r;
	}

	// Append a scalar to the beginning of a vector
	template <typename U, typename T, int N>
	toon::Vector<N+1,T> Concatenate(const U& x, const toon::Vector<N,T>& v) {
		toon::Vector<N+1,T> r;
		r.template slice<1,N>() = v;
		r[0] = x;
		return r;
	}

	// Divide the elements of a vector by the last element, as in
	// unproject(project(v)).
	template <typename T>
	inline toon::Vector<3,T> AtRetina(const toon::Vector<3,T>& x) {
		return toon::makeVector(x[0]/x[2], x[1]/x[2], 1.0);
	}

	template <typename T>
	inline toon::Vector<4,T> AtRetina(const toon::Vector<4,T>& x) {
		return toon::makeVector(x[0]/x[3], x[1]/x[3], x[2]/x[3], 1.0);
	}
}  // namespace indoor_context
