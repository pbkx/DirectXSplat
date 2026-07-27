#include "common/Math.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace directxsplat {

Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(const Vec3& a, float s) { return {a.x * s, a.y * s, a.z * s}; }
Vec3 operator/(const Vec3& a, float s) { return {a.x / s, a.y / s, a.z / s}; }

Vec3 Min(const Vec3& a, const Vec3& b) {
  return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
}

Vec3 Max(const Vec3& a, const Vec3& b) {
  return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
}

float Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

Vec3 Cross(const Vec3& a, const Vec3& b) {
  return {
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x,
  };
}

float Length(const Vec3& v) { return std::sqrt(Dot(v, v)); }

Vec3 Normalize(const Vec3& v) {
  const float len = Length(v);
  if (len <= std::numeric_limits<float>::epsilon()) {
    return {};
  }
  return v / len;
}

Quat Normalize(const Quat& q) {
  const float len = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (len <= std::numeric_limits<float>::epsilon()) {
    return {};
  }
  const float inv = 1.0f / len;
  return {q.x * inv, q.y * inv, q.z * inv, q.w * inv};
}

Mat3 QuatToMat3(const Quat& qRaw) {
  const Quat q = Normalize(qRaw);
  const float xx = q.x * q.x;
  const float yy = q.y * q.y;
  const float zz = q.z * q.z;
  const float xy = q.x * q.y;
  const float xz = q.x * q.z;
  const float yz = q.y * q.z;
  const float wx = q.w * q.x;
  const float wy = q.w * q.y;
  const float wz = q.w * q.z;

  Mat3 out{};
  out.m = {
      1.0f - 2.0f * (yy + zz), 2.0f * (xy - wz),        2.0f * (xz + wy),
      2.0f * (xy + wz),        1.0f - 2.0f * (xx + zz), 2.0f * (yz - wx),
      2.0f * (xz - wy),        2.0f * (yz + wx),        1.0f - 2.0f * (xx + yy),
  };
  return out;
}

Mat3 Diagonal3(const Vec3& d) {
  Mat3 out{};
  out.m = {d.x, 0.0f, 0.0f, 0.0f, d.y, 0.0f, 0.0f, 0.0f, d.z};
  return out;
}

Mat3 Mat3Mul(const Mat3& a, const Mat3& b) {
  Mat3 out{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out.m[r * 3 + c] = a.m[r * 3 + 0] * b.m[0 * 3 + c] + a.m[r * 3 + 1] * b.m[1 * 3 + c] +
                         a.m[r * 3 + 2] * b.m[2 * 3 + c];
    }
  }
  return out;
}

Mat3 Mat3Transpose(const Mat3& a) {
  Mat3 out{};
  out.m = {
      a.m[0], a.m[3], a.m[6],
      a.m[1], a.m[4], a.m[7],
      a.m[2], a.m[5], a.m[8],
  };
  return out;
}

Mat4 Identity4() {
  Mat4 out{};
  out.m = {
      1.0f, 0.0f, 0.0f, 0.0f,
      0.0f, 1.0f, 0.0f, 0.0f,
      0.0f, 0.0f, 1.0f, 0.0f,
      0.0f, 0.0f, 0.0f, 1.0f,
  };
  return out;
}

Mat4 Mul(const Mat4& a, const Mat4& b) {
  Mat4 out{};
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      out.m[r * 4 + c] = a.m[r * 4 + 0] * b.m[0 * 4 + c] + a.m[r * 4 + 1] * b.m[1 * 4 + c] +
                         a.m[r * 4 + 2] * b.m[2 * 4 + c] + a.m[r * 4 + 3] * b.m[3 * 4 + c];
    }
  }
  return out;
}

Vec4 Mul(const Mat4& m, const Vec4& v) {
  return {
      m.m[0] * v.x + m.m[1] * v.y + m.m[2] * v.z + m.m[3] * v.w,
      m.m[4] * v.x + m.m[5] * v.y + m.m[6] * v.z + m.m[7] * v.w,
      m.m[8] * v.x + m.m[9] * v.y + m.m[10] * v.z + m.m[11] * v.w,
      m.m[12] * v.x + m.m[13] * v.y + m.m[14] * v.z + m.m[15] * v.w,
  };
}

Mat4 Perspective(float fovYRadians, float aspect, float zNear, float zFar) {
  const float yScale = 1.0f / std::tan(fovYRadians * 0.5f);
  const float xScale = yScale / aspect;
  const float zRange = zFar - zNear;

  Mat4 out{};
  out.m = {
      xScale, 0.0f,   0.0f,                 0.0f,
      0.0f,   -yScale, 0.0f,                0.0f,
      0.0f,   0.0f,   zFar / zRange,        -zNear * zFar / zRange,
      0.0f,   0.0f,   1.0f,                 0.0f,
  };
  return out;
}

Mat4 LookAt(const Vec3& eye, const Vec3& target, const Vec3& up) {
  const Vec3 z = Normalize(target - eye);
  const Vec3 x = Normalize(Cross(up, z));
  const Vec3 y = Cross(z, x);

  Mat4 out{};
  out.m = {
      x.x, x.y, x.z, -Dot(x, eye),
      y.x, y.y, y.z, -Dot(y, eye),
      z.x, z.y, z.z, -Dot(z, eye),
      0.0f, 0.0f, 0.0f, 1.0f,
  };
  return out;
}

Mat4 Inverse(const Mat4& m) {
  const float* a = m.m.data();
  Mat4 inv{};
  float* o = inv.m.data();

  o[0] = a[5] * a[10] * a[15] - a[5] * a[11] * a[14] - a[9] * a[6] * a[15] + a[9] * a[7] * a[14] +
         a[13] * a[6] * a[11] - a[13] * a[7] * a[10];
  o[4] = -a[4] * a[10] * a[15] + a[4] * a[11] * a[14] + a[8] * a[6] * a[15] - a[8] * a[7] * a[14] -
         a[12] * a[6] * a[11] + a[12] * a[7] * a[10];
  o[8] = a[4] * a[9] * a[15] - a[4] * a[11] * a[13] - a[8] * a[5] * a[15] + a[8] * a[7] * a[13] +
         a[12] * a[5] * a[11] - a[12] * a[7] * a[9];
  o[12] = -a[4] * a[9] * a[14] + a[4] * a[10] * a[13] + a[8] * a[5] * a[14] - a[8] * a[6] * a[13] -
          a[12] * a[5] * a[10] + a[12] * a[6] * a[9];

  o[1] = -a[1] * a[10] * a[15] + a[1] * a[11] * a[14] + a[9] * a[2] * a[15] - a[9] * a[3] * a[14] -
         a[13] * a[2] * a[11] + a[13] * a[3] * a[10];
  o[5] = a[0] * a[10] * a[15] - a[0] * a[11] * a[14] - a[8] * a[2] * a[15] + a[8] * a[3] * a[14] +
         a[12] * a[2] * a[11] - a[12] * a[3] * a[10];
  o[9] = -a[0] * a[9] * a[15] + a[0] * a[11] * a[13] + a[8] * a[1] * a[15] - a[8] * a[3] * a[13] -
         a[12] * a[1] * a[11] + a[12] * a[3] * a[9];
  o[13] = a[0] * a[9] * a[14] - a[0] * a[10] * a[13] - a[8] * a[1] * a[14] + a[8] * a[2] * a[13] +
          a[12] * a[1] * a[10] - a[12] * a[2] * a[9];

  o[2] = a[1] * a[6] * a[15] - a[1] * a[7] * a[14] - a[5] * a[2] * a[15] + a[5] * a[3] * a[14] +
         a[13] * a[2] * a[7] - a[13] * a[3] * a[6];
  o[6] = -a[0] * a[6] * a[15] + a[0] * a[7] * a[14] + a[4] * a[2] * a[15] - a[4] * a[3] * a[14] -
         a[12] * a[2] * a[7] + a[12] * a[3] * a[6];
  o[10] = a[0] * a[5] * a[15] - a[0] * a[7] * a[13] - a[4] * a[1] * a[15] + a[4] * a[3] * a[13] +
          a[12] * a[1] * a[7] - a[12] * a[3] * a[5];
  o[14] = -a[0] * a[5] * a[14] + a[0] * a[6] * a[13] + a[4] * a[1] * a[14] - a[4] * a[2] * a[13] -
          a[12] * a[1] * a[6] + a[12] * a[2] * a[5];

  o[3] = -a[1] * a[6] * a[11] + a[1] * a[7] * a[10] + a[5] * a[2] * a[11] - a[5] * a[3] * a[10] -
         a[9] * a[2] * a[7] + a[9] * a[3] * a[6];
  o[7] = a[0] * a[6] * a[11] - a[0] * a[7] * a[10] - a[4] * a[2] * a[11] + a[4] * a[3] * a[10] +
         a[8] * a[2] * a[7] - a[8] * a[3] * a[6];
  o[11] = -a[0] * a[5] * a[11] + a[0] * a[7] * a[9] + a[4] * a[1] * a[11] - a[4] * a[3] * a[9] -
          a[8] * a[1] * a[7] + a[8] * a[3] * a[5];
  o[15] = a[0] * a[5] * a[10] - a[0] * a[6] * a[9] - a[4] * a[1] * a[10] + a[4] * a[2] * a[9] +
          a[8] * a[1] * a[6] - a[8] * a[2] * a[5];

  float det = a[0] * o[0] + a[1] * o[4] + a[2] * o[8] + a[3] * o[12];
  if (std::abs(det) < 1e-8f) {
    return Identity4();
  }
  det = 1.0f / det;
  for (auto& e : inv.m) {
    e *= det;
  }
  return inv;
}

float Clamp(float v, float lo, float hi) { return std::max(lo, std::min(v, hi)); }

Mat3 BuildCovariance(const Vec3& scale, const Quat& rotation) {
  const Mat3 r = QuatToMat3(rotation);
  const Vec3 sq{scale.x * scale.x, scale.y * scale.y, scale.z * scale.z};
  const Mat3 s = Diagonal3(sq);
  return Mat3Mul(Mat3Mul(r, s), Mat3Transpose(r));
}

Mat3 ProjectCovarianceToScreen(const Mat3& covariance,
                               const Vec3& positionView,
                               float focalX,
                               float focalY) {
  const float viewZSign = positionView.z < 0.0f ? -1.0f : 1.0f;
  const float viewDepth = std::max(std::abs(positionView.z), 1e-4f);
  const float invZ = 1.0f / viewDepth;
  const float invZ2 = invZ * invZ;

  const float j00 = focalX * invZ;
  const float j02 = -viewZSign * focalX * positionView.x * invZ2;
  const float j11 = focalY * invZ;
  const float j12 = -viewZSign * focalY * positionView.y * invZ2;

  const float c00 = covariance.m[0];
  const float c01 = 0.5f * (covariance.m[1] + covariance.m[3]);
  const float c02 = 0.5f * (covariance.m[2] + covariance.m[6]);
  const float c11 = covariance.m[4];
  const float c12 = 0.5f * (covariance.m[5] + covariance.m[7]);
  const float c22 = covariance.m[8];

  const float a = j00 * j00 * c00 + 2.0f * j00 * j02 * c02 + j02 * j02 * c22;
  const float b = j00 * j11 * c01 + j00 * j12 * c02 + j02 * j11 * c12 + j02 * j12 * c22;
  const float c = j11 * j11 * c11 + 2.0f * j11 * j12 * c12 + j12 * j12 * c22;

  Mat3 out{};
  out.m = {
      std::max(a, 1e-6f), b,               0.0f,
      b,               std::max(c, 1e-6f), 0.0f,
      0.0f,            0.0f,            1.0f,
  };
  return out;
}

}  // namespace directxsplat
