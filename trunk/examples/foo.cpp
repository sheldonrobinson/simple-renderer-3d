#include "matrix_types.h"
#include "simple_renderer.h"

#include "vector_utils.tpp"

using namespace indoor_context;
using namespace Eigen;

int main(int argc, char **argv) {
	Vec2I viewport = MakeVector(100,100);
	LinearCamera camera;

	SimpleRenderer re(camera, viewport);

	Vec3 p(10, 10, 0);
	Vec3 q(10, 90, 0);
	Vec3 r(90, 10, 0);

	re.Render(p, q, r, 1);

	return 0;
}
