#pragma once

#include "matrix_types.h"

namespace indoor_context {
	class SimpleRenderer {
	public:
		// Make sure we're aligned (since we have eigen members)
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		// Initialize empty
		SimpleRenderer();
		// Initialize with the given camera
		//SimpleRenderer(const PosedCamera& cam);
		// Initialize with the given camera and viewport
		SimpleRenderer(const LinearCamera& cam, Vec2I viewport);

		// Get the frame buffer
		const Eigen::ArrayXXi& framebuffer() const { return framebuffer_; }
		Eigen::ArrayXXi& framebuffer() { return framebuffer_; }
		// Get the depth buffer
		const Eigen::ArrayXXd& depthbuffer() const { return depthbuffer_; }
		Eigen::ArrayXXd& depthbuffer() { return depthbuffer_; }
		// Get the camera
		const LinearCamera& camera() const { return camera_; }
		// Get the viewport
		const Vec2I& viewport() const { return viewport_; }

		// Configure the renderer with the given camera and viewport
		void Configure(const LinearCamera& cam, Vec2I viewport);
		// Clear all buffers
		void Clear(int bg);

		// Render a triangle. Return true if at least one pixel was affected.
		bool Render(const Vec2& p, const Vec2& q, const Vec2& r, int label);
		// Render a triangle (homogeneous coords). Return true if at least
		// one pixel was affected.
		bool Render(const Vec3& p, const Vec3& q, const Vec3& r, int label);
		// Render an infinite plane z=z0. Internally we just use very large extents.
		bool OldRenderInfinitePlane(double z0, int label);
		// Render an infinite plane z=z0. Internally we just use very large extents.
		bool RenderInfinitePlane(double z0, int label);

		// We often get NaN/inf pixels when there are walls close to the
		// horizon, or due to clipping issues near the boundary of the
		// image. As a simple work around we replace any such values by the
		// max finite value (which are clamped to kClampDepth). Returns the
		// number of pixels modified.
		int SmoothInfiniteDepths();
	private:
		Vec2I viewport_;
		LinearCamera camera_;
		Eigen::ArrayXXi framebuffer_;
		Eigen::ArrayXXd depthbuffer_;
	};
}  // namespace indoor_context
