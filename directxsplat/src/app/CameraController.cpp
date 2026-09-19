#include "app/CameraController.h"

#include <algorithm>
#include <cmath>

#include "api/CameraSetInternal.h"

namespace directxsplat {

namespace {

constexpr float kMaxCameraDt = 0.05f;
constexpr float kMaxLookDelta = 240.0f;
constexpr float kMaxWheelDelta = 10.0f;
constexpr float kTwoPi = 6.28318530718f;

bool Finite(float v) {
  return std::isfinite(v);
}

bool Finite(const Vec3& v) {
  return Finite(v.x) && Finite(v.y) && Finite(v.z);
}

bool Finite(const Quat& q) {
  return Finite(q.x) && Finite(q.y) && Finite(q.z) && Finite(q.w);
}

float ClampFinite(float v, float lo, float hi, float fallback) {
  return Finite(v) ? std::clamp(v, lo, hi) : fallback;
}

float WrapAngle(float v) {
  return Finite(v) ? std::remainder(v, kTwoPi) : 0.0f;
}

Vec3 SafeNormalize(const Vec3& v, const Vec3& fallback) {
  if (!Finite(v)) {
    return fallback;
  }
  const float len = Length(v);
  if (!Finite(len) || len <= 1e-6f) {
    return fallback;
  }
  return v / len;
}

Quat QuaternionFromYawPitch(float yaw, float pitch) {
  const float cy = std::cos(yaw * 0.5f);
  const float sy = std::sin(yaw * 0.5f);
  const float cp = std::cos(pitch * 0.5f);
  const float sp = std::sin(pitch * 0.5f);

  return Normalize({
      sp * cy,
      cp * sy,
      -sp * sy,
      cp * cy,
  });
}

Vec3 RotateVector(const Quat& q, const Vec3& v) {
  const Vec3 u{q.x, q.y, q.z};
  const float s = q.w;
  return u * (2.0f * Dot(u, v)) + v * (s * s - Dot(u, u)) + Cross(u, v) * (2.0f * s);
}

void BuildViewBasis(const Vec3& forward, float roll, Vec3& right, Vec3& up) {
  const Vec3 f = SafeNormalize(forward, {0.0f, 0.0f, 1.0f});
  const Vec3 referenceUp = std::abs(f.y) > 0.98f ? Vec3{0.0f, 0.0f, 1.0f} : Vec3{0.0f, 1.0f, 0.0f};
  const Vec3 baseRight = SafeNormalize(Cross(referenceUp, f), {1.0f, 0.0f, 0.0f});
  const Vec3 baseUp = SafeNormalize(Cross(f, baseRight), {0.0f, 1.0f, 0.0f});
  const float safeRoll = WrapAngle(roll);
  const float c = std::cos(safeRoll);
  const float s = std::sin(safeRoll);
  right = SafeNormalize(baseRight * c + baseUp * s, baseRight);
  up = SafeNormalize(baseUp * c - baseRight * s, baseUp);
}

}  

CameraController::CameraController() = default;

void CameraController::ClearMatrixOverride() {
  matrixOverride_.reset();
}

void CameraController::SetViewport(uint32_t width, uint32_t height) {
  viewportWidth_ = std::max(width, 1u);
  viewportHeight_ = std::max(height, 1u);
}

void CameraController::SetState(const CameraState& state) {
  ClearMatrixOverride();
  state_ = state;
  if (!Finite(state_.position)) {
    state_.position = {};
  }
  if (!Finite(state_.yaw)) {
    state_.yaw = 0.0f;
  }
  if (!Finite(state_.pitch)) {
    state_.pitch = 0.0f;
  }
  if (!Finite(state_.roll)) {
    state_.roll = 0.0f;
  }
  state_.yaw = WrapAngle(state_.yaw);
  state_.roll = WrapAngle(state_.roll);
  if (state_.navigatorMode == NavigatorMode::Trackball) {
    state_.navigatorMode = NavigatorMode::Orbit;
  }
  state_.fovYRadians = Finite(state_.fovYRadians) ? std::clamp(state_.fovYRadians, 0.017453292f, 3.12413936f)
                                                   : kDefaultCameraFovYRadians;
  state_.nearPlane = Finite(state_.nearPlane) ? std::max(state_.nearPlane, 0.0001f) : kDefaultCameraNearPlane;
  state_.farPlane = Finite(state_.farPlane) ? std::max(state_.farPlane, state_.nearPlane + 0.001f)
                                             : kDefaultCameraFarPlane;
  state_.movementSpeed = Finite(state_.movementSpeed) ? std::max(state_.movementSpeed, 0.0f) : 2.5f;
  state_.rotationSpeed = Finite(state_.rotationSpeed) ? std::max(state_.rotationSpeed, 0.0f) : 1.0f;
  if (!Finite(state_.orbitPivot)) {
    state_.orbitPivot = {};
  }
  state_.orbitDistance = Finite(state_.orbitDistance) ? std::max(state_.orbitDistance, 0.01f) : 2.0f;
  ClampPitch();
}

const CameraState& CameraController::State() const { return state_; }

bool CameraController::HasMatrixOverride() const { return matrixOverride_.has_value(); }

void CameraController::UpdateFps(float dt, bool moveForward, bool moveBackward, bool moveLeft, bool moveRight,
                                 bool moveUp, bool moveDown, float lookDeltaX, float lookDeltaY,
                                 float rollDelta, bool rotationEnabled, float movementSpeedMultiplier) {
  dt = ClampFinite(dt, 0.0f, kMaxCameraDt, 0.0f);
  movementSpeedMultiplier = Finite(movementSpeedMultiplier) ? std::max(movementSpeedMultiplier, 0.0f) : 1.0f;
  lookDeltaX = ClampFinite(lookDeltaX, -kMaxLookDelta, kMaxLookDelta, 0.0f);
  lookDeltaY = ClampFinite(lookDeltaY, -kMaxLookDelta, kMaxLookDelta, 0.0f);
  rollDelta = ClampFinite(rollDelta, -kMaxLookDelta, kMaxLookDelta, 0.0f);
  const bool movingInput = moveForward || moveBackward || moveLeft || moveRight || moveUp || moveDown;
  if (rotationEnabled || movingInput) {
    ClearMatrixOverride();
  }

  if (rotationEnabled) {
    state_.yaw += lookDeltaX * 0.002f * state_.rotationSpeed;
    state_.pitch += lookDeltaY * 0.002f * state_.rotationSpeed;
    state_.roll += rollDelta * 0.002f * state_.rotationSpeed;
    state_.yaw = WrapAngle(state_.yaw);
    state_.roll = WrapAngle(state_.roll);
    ClampPitch();
  }

  const Vec3 f = Forward();
  const Vec3 r = Right();
  const Vec3 u{0.0f, 1.0f, 0.0f};

  Vec3 delta{};
  if (moveForward) delta = delta + f;
  if (moveBackward) delta = delta - f;
  if (moveRight) delta = delta + r;
  if (moveLeft) delta = delta - r;
  if (moveUp) delta = delta + u;
  if (moveDown) delta = delta - u;

  const bool moving = Length(delta) > 0.0f;
  if (state_.useAcceleration) {
    accelerationFactor_ = moving ? std::min(accelerationFactor_ * 1.02f, 50.0f) : 1.0f;
  } else {
    accelerationFactor_ = 1.0f;
  }

  if (moving) {
    delta = Normalize(delta) * (state_.movementSpeed * movementSpeedMultiplier * accelerationFactor_ * dt);
    state_.position = state_.position + delta;
    if (!Finite(state_.position)) {
      state_.position = {};
    }
  }

  if (state_.navigatorMode == NavigatorMode::Trackball || state_.navigatorMode == NavigatorMode::Orbit) {
    state_.orbitPivot = state_.position + f * state_.orbitDistance;
  }
}

void CameraController::UpdateOrbit(float, float orbitDeltaX, float orbitDeltaY, float panDeltaX, float panDeltaY,
                                    float wheelDelta) {
  orbitDeltaX = ClampFinite(orbitDeltaX, -kMaxLookDelta, kMaxLookDelta, 0.0f);
  orbitDeltaY = ClampFinite(orbitDeltaY, -kMaxLookDelta, kMaxLookDelta, 0.0f);
  panDeltaX = ClampFinite(panDeltaX, -kMaxLookDelta, kMaxLookDelta, 0.0f);
  panDeltaY = ClampFinite(panDeltaY, -kMaxLookDelta, kMaxLookDelta, 0.0f);
  wheelDelta = ClampFinite(wheelDelta, -kMaxWheelDelta, kMaxWheelDelta, 0.0f);
  if (orbitDeltaX != 0.0f || orbitDeltaY != 0.0f || panDeltaX != 0.0f || panDeltaY != 0.0f || wheelDelta != 0.0f) {
    ClearMatrixOverride();
  }
  state_.yaw += orbitDeltaX * 0.002f * state_.rotationSpeed;
  state_.pitch += orbitDeltaY * 0.002f * state_.rotationSpeed;
  state_.yaw = WrapAngle(state_.yaw);
  ClampPitch();

  const Vec3 r = Right();
  const Vec3 u = Up();
  state_.orbitPivot = state_.orbitPivot - r * (panDeltaX * 0.01f) + u * (panDeltaY * 0.01f);

  state_.orbitDistance = std::max(0.01f, state_.orbitDistance * std::exp(-wheelDelta * 0.1f));
  state_.position = state_.orbitPivot - Forward() * state_.orbitDistance;
}

void CameraController::UpdateTrackball(float, float orbitDeltaX, float orbitDeltaY, float panDeltaX, float panDeltaY,
                                        float wheelDelta) {
  orbitDeltaX = ClampFinite(orbitDeltaX, -kMaxLookDelta, kMaxLookDelta, 0.0f);
  orbitDeltaY = ClampFinite(orbitDeltaY, -kMaxLookDelta, kMaxLookDelta, 0.0f);
  panDeltaX = ClampFinite(panDeltaX, -kMaxLookDelta, kMaxLookDelta, 0.0f);
  panDeltaY = ClampFinite(panDeltaY, -kMaxLookDelta, kMaxLookDelta, 0.0f);
  wheelDelta = ClampFinite(wheelDelta, -kMaxWheelDelta, kMaxWheelDelta, 0.0f);
  if (orbitDeltaX != 0.0f || orbitDeltaY != 0.0f || panDeltaX != 0.0f || panDeltaY != 0.0f || wheelDelta != 0.0f) {
    ClearMatrixOverride();
  }
  const Vec3 before = SafeNormalize(state_.position - state_.orbitPivot, Forward() * -1.0f);
  state_.yaw += orbitDeltaX * 0.002f * state_.rotationSpeed;
  state_.pitch += orbitDeltaY * 0.002f * state_.rotationSpeed;
  state_.roll += (orbitDeltaX * orbitDeltaY) * 0.000002f * state_.rotationSpeed;
  state_.yaw = WrapAngle(state_.yaw);
  state_.roll = WrapAngle(state_.roll);
  ClampPitch();

  const Vec3 r = Right();
  const Vec3 u = Up();
  const float panScale = std::max(0.001f, state_.orbitDistance * 0.002f);
  state_.orbitPivot = state_.orbitPivot - r * (panDeltaX * panScale) + u * (panDeltaY * panScale);
  state_.orbitDistance = std::max(0.01f, state_.orbitDistance * std::exp(-wheelDelta * 0.12f));
  const Vec3 after = SafeNormalize(before + Right() * (-orbitDeltaX * 0.002f) + Up() * (orbitDeltaY * 0.002f), before);
  state_.position = state_.orbitPivot + after * state_.orbitDistance;
  const Vec3 f = SafeNormalize(state_.orbitPivot - state_.position, Forward());
  state_.yaw = WrapAngle(std::atan2(f.x, f.z));
  state_.pitch = -std::asin(std::clamp(f.y, -1.0f, 1.0f));
}

void CameraController::FocusBounds(const Aabb& bounds) {
  if (!bounds.valid) {
    return;
  }
  const Vec3 center = ComputeAabbCenter(bounds);
  const float radius = std::max(ComputeAabbRadius(bounds), 0.01f);
  FocusPoint(center, radius);
}

void CameraController::FocusPoint(const Vec3& point, float estimatedRadius) {
  ClearMatrixOverride();
  const float radius = std::max(estimatedRadius, 0.01f);
  state_.orbitPivot = point;
  const float halfFovY = state_.fovYRadians * 0.5f;
  const float aspect = static_cast<float>(std::max(viewportWidth_, 1u)) / static_cast<float>(std::max(viewportHeight_, 1u));
  const float halfFovX = std::atan(std::tan(halfFovY) * std::max(aspect, 0.01f));
  const float fitHalfFov = std::clamp(std::min(halfFovX, halfFovY), 0.05f, 1.5f);
  state_.orbitDistance = (radius / std::sin(fitHalfFov)) * 1.30f;
  if (state_.navigatorMode == NavigatorMode::Trackball || state_.navigatorMode == NavigatorMode::Orbit ||
      state_.navigatorMode == NavigatorMode::Interpolation) {
    state_.position = state_.orbitPivot - Forward() * state_.orbitDistance;
  } else {
    state_.position = point - Forward() * state_.orbitDistance;
  }
}

void CameraController::SetOrbitPivot(const Vec3& pivot) {
  ClearMatrixOverride();
  state_.orbitPivot = pivot;
  if (state_.navigatorMode == NavigatorMode::Trackball || state_.navigatorMode == NavigatorMode::Orbit ||
      state_.navigatorMode == NavigatorMode::Interpolation) {
    state_.orbitDistance = std::max(0.01f, Length(state_.position - state_.orbitPivot));
  }
}

void CameraController::SnapToInputCamera(const InputCamera& camera) {
  ClearMatrixOverride();
  if (!Finite(camera.position) || !Finite(camera.rotation)) {
    return;
  }
  const Quat q = Normalize(camera.rotation);
  if (!Finite(q) || (q.x == 0.0f && q.y == 0.0f && q.z == 0.0f && q.w == 0.0f)) {
    return;
  }
  const Vec3 f = RotateVector(q, {0.0f, 0.0f, 1.0f});
  const Vec3 inputUp = Normalize(RotateVector(q, {0.0f, 1.0f, 0.0f}));
  if (!Finite(f) || !Finite(inputUp) || Length(f) <= 1e-6f || Length(inputUp) <= 1e-6f) {
    return;
  }
  SetPoseFromForwardUp(camera.position, f, inputUp);
}

void CameraController::SnapToCameraParams(const CameraParams& camera) {
  if (!ValidateCameraParamsForRendering(camera).ok) {
    return;
  }

  const CameraRenderState renderState = CameraRenderStateFromCameraParams(camera, state_.nearPlane, state_.farPlane);
  const auto& e = camera.extrinsic;
  const Vec3 forward = Normalize(Vec3{
      e[8],
      e[9],
      e[10],
  });
  const Vec3 poseUp = Normalize(CameraPoseUpFromExtrinsic(e));
  if (!Finite(renderState.position) || !Finite(forward) || !Finite(poseUp) ||
      Length(forward) <= 1e-6f || Length(poseUp) <= 1e-6f) {
    return;
  }

  SetPoseFromForwardUp(renderState.position, forward, poseUp);
  matrixOverride_ = CameraMatrixOverride{};
  matrixOverride_->view = renderState.view;
  matrixOverride_->position = renderState.position;
}

bool CameraController::SnapToClosestInputCamera(const std::vector<InputCamera>& cameras) {
  if (cameras.empty()) {
    return false;
  }

  size_t best = 0;
  float bestDist2 = std::numeric_limits<float>::max();
  for (size_t i = 0; i < cameras.size(); ++i) {
    const Vec3 d = cameras[i].position - state_.position;
    const float dist2 = Dot(d, d);
    if (dist2 < bestDist2) {
      bestDist2 = dist2;
      best = i;
    }
  }

  SnapToInputCamera(cameras[best]);
  return true;
}

Mat4 CameraController::ViewMatrix() const {
  if (matrixOverride_.has_value()) {
    return matrixOverride_->view;
  }
  return LookAt(state_.position, state_.position + Forward(), Up());
}

Mat4 CameraController::ProjectionMatrix() const {
  const float aspect = static_cast<float>(viewportWidth_) / static_cast<float>(viewportHeight_);
  return ProjectionMatrixForAspect(aspect);
}

Mat4 CameraController::ProjectionMatrixForAspect(float aspect) const {
  return Perspective(state_.fovYRadians, std::max(aspect, 0.01f), state_.nearPlane, state_.farPlane);
}

Vec3 CameraController::Forward() const {
  const Quat q = QuaternionFromYawPitch(state_.yaw, state_.pitch);
  return SafeNormalize(RotateVector(q, {0.0f, 0.0f, 1.0f}), {0.0f, 0.0f, 1.0f});
}

Vec3 CameraController::Right() const {
  Vec3 right{};
  Vec3 up{};
  BuildViewBasis(Forward(), state_.roll, right, up);
  return right;
}

Vec3 CameraController::Up() const {
  Vec3 right{};
  Vec3 up{};
  BuildViewBasis(Forward(), state_.roll, right, up);
  return up;
}

Vec3 CameraController::ScreenToWorldRayDir(float ndcX, float ndcY) const {
  Mat4 invProj = Inverse(ProjectionMatrix());
  Mat4 invView = Inverse(ViewMatrix());
  Vec4 clip{ndcX, ndcY, 1.0f, 1.0f};
  Vec4 view = Mul(invProj, clip);
  if (std::abs(view.w) > 1e-6f) {
    view.x /= view.w;
    view.y /= view.w;
    view.z /= view.w;
  }
  view.w = 0.0f;
  Vec4 world = Mul(invView, view);
  return SafeNormalize(Vec3{world.x, world.y, world.z}, Forward());
}

void CameraController::SetPoseFromForwardUp(const Vec3& position, const Vec3& forward, const Vec3& up) {
  if (!Finite(position) || !Finite(forward) || !Finite(up) || Length(forward) <= 1e-6f || Length(up) <= 1e-6f) {
    return;
  }

  const Vec3 f = SafeNormalize(forward, {0.0f, 0.0f, 1.0f});
  const Vec3 inputUp = SafeNormalize(up, {0.0f, 1.0f, 0.0f});
  state_.position = position;
  state_.yaw = WrapAngle(std::atan2(f.x, f.z));
  state_.pitch = -std::asin(std::clamp(f.y, -1.0f, 1.0f));
  Vec3 baseRight{};
  Vec3 baseUp{};
  BuildViewBasis(Forward(), 0.0f, baseRight, baseUp);
  state_.roll = WrapAngle(std::atan2(-Dot(inputUp, baseRight), Dot(inputUp, baseUp)));
  state_.fovYRadians = kDefaultCameraFovYRadians;
  state_.orbitPivot = state_.position + f * state_.orbitDistance;
  SetState(state_);
}

void CameraController::ClampPitch() { state_.pitch = std::clamp(state_.pitch, -1.55334f, 1.55334f); }

}  // namespace directxsplat
