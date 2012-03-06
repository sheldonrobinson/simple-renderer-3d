#include "common_types.h"
#include "simple_renderer.h"

using namespace indoor_context;
using namespace toon;

int main(int argc, char **argv) {
	Vec2I viewport = makeVector(100,100);
	Matrix<3,4> camera;

	SimpleRenderer re(camera, viewport);

	Vec3 p = makeVector(10,10,0);
	Vec3 q = makeVector(10,90,0);
	Vec3 r = makeVector(90,10,0);

	re.Render(p, q, r, 1);

	return 0;
}
