#include <doctest/doctest.h>

#include <cmath>

#include "api/CameraSetInternal.h"
#include "app/CameraPathAnimator.h"
#include "directxsplat/math.h"

namespace {

directxsplat::CameraParams MakeCamera(float x, float y, float z) {
  directxsplat::CameraParams camera{};
  camera.width = 100;
  camera.height = 100;
  camera.intrinsic = {
      100.0f, 0.0f, 50.0f,
      0.0f, 100.0f, 50.0f,
      0.0f, 0.0f, 1.0f,
  };
  camera.extrinsic = {
      1.0f, 0.0f, 0.0f, -x,
      0.0f, -1.0f, 0.0f, y,
      0.0f, 0.0f, 1.0f, -z,
      0.0f, 0.0f, 0.0f, 1.0f,
  };
  return camera;
}

directxsplat::CameraParams MakeLoadedCamera() {
  directxsplat::CameraParams camera{};
  camera.width = 800;
  camera.height = 600;
  camera.intrinsic = {
      100.0f, 0.0f, 400.0f,
      0.0f, 200.0f, 300.0f,
      0.0f, 0.0f, 1.0f,
  };
  camera.extrinsic = {
      1.0f, 0.0f, 0.0f, -1.0f,
      0.0f, 0.8660254f, 0.5f, -3.2320508f,
      0.0f, -0.5f, 0.8660254f, -1.5980762f,
      0.0f, 0.0f, 0.0f, 1.0f,
  };
  return camera;
}

directxsplat::CameraSet MakeCameraSet(size_t count) {
  directxsplat::CameraSet cameras{};
  for (size_t i = 0; i < count; ++i) {
    cameras.cameras.push_back(MakeCamera(static_cast<float>(i), static_cast<float>(i % 2u), 2.0f));
  }
  return cameras;
}

bool Finite(const directxsplat::CameraState& state) {
  return std::isfinite(state.position.x) && std::isfinite(state.position.y) && std::isfinite(state.position.z) &&
         std::isfinite(state.yaw) && std::isfinite(state.pitch) && std::isfinite(state.roll);
}

void CheckMatrix(const directxsplat::Mat4& actual, const directxsplat::Mat4& expected) {
  for (size_t i = 0; i < actual.m.size(); ++i) {
    CHECK(actual.m[i] == doctest::Approx(expected.m[i]));
  }
}

}

TEST_CASE("Animator with zero cameras returns false") {
  directxsplat::CameraPathAnimator animator;
  directxsplat::CameraState state{};

  CHECK_FALSE(animator.Evaluate(state));
}

TEST_CASE("Animator with one camera returns that camera") {
  directxsplat::CameraPathAnimator animator;
  animator.SetCameras(MakeCameraSet(1));

  directxsplat::CameraState state{};
  CHECK(animator.Evaluate(state));
  CHECK(state.position.x == doctest::Approx(0.0f));
  CHECK(state.position.y == doctest::Approx(0.0f));
  CHECK(state.position.z == doctest::Approx(2.0f));
}

TEST_CASE("Advance wraps by camera count") {
  directxsplat::CameraPathAnimator animator;
  animator.SetCameras(MakeCameraSet(4));
  animator.SetTime(3.75f);
  animator.Advance(1.0f, 1.0f);

  CHECK(animator.Time() >= 0.0f);
  CHECK(animator.Time() < 4.0f);
  CHECK(animator.Time() == doctest::Approx(0.75f));
}

TEST_CASE("Manual camera edit disables animation") {
  directxsplat::AnimationUiState state{};
  state.enabled = true;

  directxsplat::StopAnimationOnCameraEdit(state, true);

  CHECK_FALSE(state.enabled);
}

TEST_CASE("Interpolated pose is finite") {
  directxsplat::CameraPathAnimator animator;
  animator.SetCameras(MakeCameraSet(4));

  for (float time : {0.0f, 0.25f, 1.5f, 3.75f}) {
    animator.SetTime(time);
    directxsplat::CameraState state{};
    CHECK(animator.Evaluate(state));
    CHECK(Finite(state));
  }
}

TEST_CASE("Camera snap uses default viewer fov") {
  directxsplat::CameraController controller;
  directxsplat::CameraState state = controller.State();
  state.fovYRadians = 0.75f;
  controller.SetState(state);
  const directxsplat::CameraParams camera = MakeLoadedCamera();
  const directxsplat::CameraRenderState expected =
      directxsplat::CameraRenderStateFromCameraParams(camera, state.nearPlane, state.farPlane);

  controller.SnapToCameraParams(camera);

  CHECK(controller.State().position.x == doctest::Approx(1.0f));
  CHECK(controller.State().position.y == doctest::Approx(2.0f));
  CHECK(controller.State().position.z == doctest::Approx(3.0f));
  CHECK(controller.State().fovYRadians == doctest::Approx(directxsplat::kDefaultCameraFovYRadians));
  CHECK(controller.HasMatrixOverride());
  CheckMatrix(controller.ViewMatrix(), expected.view);
  CheckMatrix(controller.ProjectionMatrixForAspect(1.0f),
              directxsplat::Perspective(directxsplat::kDefaultCameraFovYRadians, 1.0f,
                                         controller.State().nearPlane, controller.State().farPlane));

  controller.UpdateFps(0.0f, false, false, false, false, false, false, 0.0f, 0.0f, 0.0f, true);

  CHECK_FALSE(controller.HasMatrixOverride());
  CheckMatrix(controller.ViewMatrix(), expected.view);
}

TEST_CASE("FPS movement multiplier scales translation") {
  directxsplat::CameraController normal;
  directxsplat::CameraController fast;
  directxsplat::CameraState state = normal.State();
  state.movementSpeed = 2.0f;
  normal.SetState(state);
  fast.SetState(state);

  normal.UpdateFps(0.02f, true, false, false, false, false, false, 0.0f, 0.0f, 0.0f, false, 1.0f);
  fast.UpdateFps(0.02f, true, false, false, false, false, false, 0.0f, 0.0f, 0.0f, false, 4.0f);

  CHECK(fast.State().position.x == doctest::Approx(normal.State().position.x * 4.0f));
  CHECK(fast.State().position.y == doctest::Approx(normal.State().position.y * 4.0f));
  CHECK(fast.State().position.z == doctest::Approx(normal.State().position.z * 4.0f));
}

TEST_CASE("Animator preserves loaded camera basis") {
  directxsplat::CameraSet cameras{};
  cameras.cameras.push_back(MakeLoadedCamera());
  directxsplat::CameraPathAnimator animator;
  animator.SetCameras(cameras);

  directxsplat::CameraState state{};
  REQUIRE(animator.Evaluate(state));

  directxsplat::CameraController controller;
  controller.SetState(state);
  const directxsplat::CameraRenderState expected =
      directxsplat::CameraRenderStateFromCameraParams(cameras.cameras[0], state.nearPlane, state.farPlane);
  CheckMatrix(controller.ViewMatrix(), expected.view);
}
