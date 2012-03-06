#pragma once

#include <string>
#include <vector>
#include <utility>
#include <exception>

#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>
#include <boost/lexical_cast.hpp>

// forward-declare namespaces in order to introduce namespace alias below
namespace boost { namespace filesystem { } }
namespace TooN { }

namespace indoor_context {
	using namespace std;

	using boost::scoped_ptr;
	using boost::scoped_array;

	namespace toon=TooN;

	typedef unsigned char byte;
}

#include "matrix_types.h"

