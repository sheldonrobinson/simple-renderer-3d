#pragma once

#include <cmath>

#include "matrix_types.h"

namespace indoor_context {
	// Round and cast to integer
	template <typename T>
	inline int Roundi(const T x) {
		return static_cast<int>(std::floor(x+.5));
	}

	// Ceil and cast to integer
	template <typename T>
	inline int Ceili(const T x) {
		return static_cast<int>(std::ceil(x));
	}

	// Floor and cast to integer
	template <typename T>
	inline int Floori(const T x) {
		return static_cast<int>(std::floor(x));
	}

	// Clamp x to [min,max]
	template <typename T, typename U, typename V>
	inline T Clamp(const T& x, const U& min, const V& max) {
		return x < min ?
			static_cast<T>(min) :
			(x > max ? static_cast<T>(max) : x);
	}

	// Returns the sign of x
	template <typename T>
	int Sign(const T x) {
		return x < 0 ? -1 : (x > 0 ? 1 : 0);
	}

	// Returns 1 if x >= 0, otherwise returns -1
	template <typename T>
	int HalfSign(T x) {
		return x < 0 ? -1 : 1;
	}
}
