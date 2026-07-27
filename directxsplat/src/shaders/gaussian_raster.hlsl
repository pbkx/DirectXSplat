struct SetParamsGpu {
  float scalingModifier;
  uint visible;
  float3 decodeMin;
  float3 decodeExtent;
  float2 pad;
};

struct ChunkPrepGpu {
  SetParamsGpu params;
  uint gaussianOffset;
  uint gaussianCount;
  uint2 pad;
};

struct RasterGaussian {
  float4 clipPos;
  float2 axisU;
  float2 axisV;
  float2 centerPixel;
  float alphaCutPower;
  float4 conicOpacity;
  float3 color;
  float viewDistance;
};

cbuffer PrepConstants : register(b0) {
  row_major float4x4 gView;
  row_major float4x4 gProj;
  float3 gCameraPos;
  float gGlobalScale;
  float gFocalX;
  float gFocalY;
  float gNdcX;
  float gNdcY;
  float gMaxAxisPixels;
  float gNearPlane;
  uint gSceneCount;
  uint gPaddedCount;
  uint gSetCount;
  uint gFastCulling;
  uint gRenderType;
  uint gAntialiasingMode;
  uint gShadingDegree;
  uint gPositiveViewSpaceZ;
  float gAntialiasingStrength;
  uint gGammaCorrection;
  uint gDrawCapacity;
  uint gPairCapacity;
  uint gViewportWidth;
  uint gViewportHeight;
  float3 gBackgroundColor;
  float gFarPlane;
  float gFrustumDilation;
  uint gSceneGaussianStride;
  uint gRgbaFormat;
  uint gShFormat;
  uint gRgbaOffset;
  uint gShOffset;
  uint gIdOffset;
  uint gPad3;
};

ByteAddressBuffer gSceneGaussians : register(t0);
StructuredBuffer<uint> gSortedSceneIndices : register(t1);
StructuredBuffer<ChunkPrepGpu> gChunkPrep : register(t2);
StructuredBuffer<uint> gSceneIndexToChunk : register(t3);

static const uint kFormatFloat32 = 0u;
static const uint kFormatFloat16 = 1u;
static const uint kFormatUint8 = 2u;
static const float kPackedShUint8Range = 4.0f;

float DecodeHalf(uint bits) {
  return f16tof32(bits & 0xFFFFu);
}

float2 DecodeHalf2(uint bits) {
  return float2(DecodeHalf(bits), DecodeHalf(bits >> 16u));
}

float DecodeSnorm16(uint bits) {
  int s = (int)(bits & 0xFFFFu);
  s = (s << 16) >> 16;
  return max((float)s / 32767.0, -1.0);
}

float DecodeUnorm16(uint bits) {
  return (float)(bits & 0xFFFFu) * (1.0 / 65535.0);
}

uint GaussianBaseByteOffset(uint index) {
  return index * gSceneGaussianStride;
}

uint LoadGaussianWord(uint index, uint wordIndex) {
  return gSceneGaussians.Load(GaussianBaseByteOffset(index) + wordIndex * 4u);
}

uint LoadGaussianByteAt(uint index, uint byteOffset) {
  const uint absolute = GaussianBaseByteOffset(index) + byteOffset;
  const uint word = gSceneGaussians.Load(absolute & ~3u);
  return (word >> ((absolute & 3u) * 8u)) & 0xFFu;
}

uint LoadGaussianU16At(uint index, uint byteOffset) {
  const uint absolute = GaussianBaseByteOffset(index) + byteOffset;
  const uint word = gSceneGaussians.Load(absolute & ~3u);
  return (word >> ((absolute & 2u) * 8u)) & 0xFFFFu;
}

float LoadPackedAttribute(uint index, uint byteOffset, uint format, bool shCoefficient) {
  if (format == kFormatFloat32) {
    return asfloat(gSceneGaussians.Load(GaussianBaseByteOffset(index) + byteOffset));
  }
  if (format == kFormatUint8) {
    const float u = (float)LoadGaussianByteAt(index, byteOffset) * (1.0f / 255.0f);
    return shCoefficient ? ((u * 2.0f - 1.0f) * kPackedShUint8Range) : u;
  }
  return DecodeHalf(LoadGaussianU16At(index, byteOffset));
}

uint AttributeBytes(uint format) {
  if (format == kFormatFloat32) {
    return 4u;
  }
  if (format == kFormatUint8) {
    return 1u;
  }
  return 2u;
}

float4 DecodeSceneGaussianRgba(uint index) {
  const uint elementBytes = AttributeBytes(gRgbaFormat);
  return float4(
      LoadPackedAttribute(index, gRgbaOffset + 0u * elementBytes, gRgbaFormat, true),
      LoadPackedAttribute(index, gRgbaOffset + 1u * elementBytes, gRgbaFormat, true),
      LoadPackedAttribute(index, gRgbaOffset + 2u * elementBytes, gRgbaFormat, true),
      LoadPackedAttribute(index, gRgbaOffset + 3u * elementBytes, gRgbaFormat, false));
}

void DecodeSceneGaussianBase(uint index,
                             SetParamsGpu sp,
                             out float3 position,
                             out float opacity,
                             out float3 scale,
                             out float filterScale,
                             out float4 rotation) {
  uint word0 = LoadGaussianWord(index, 0u);
  uint word1 = LoadGaussianWord(index, 1u);
  uint word2 = LoadGaussianWord(index, 2u);
  uint word3 = LoadGaussianWord(index, 3u);
  uint word4 = LoadGaussianWord(index, 4u);
  uint word5 = LoadGaussianWord(index, 5u);

  float3 pos01 = float3(DecodeUnorm16(word0), DecodeUnorm16(word0 >> 16u), DecodeUnorm16(word1));
  position = sp.decodeMin + sp.decodeExtent * pos01;
  opacity = saturate(DecodeSceneGaussianRgba(index).w);
  float2 scaleXY = DecodeHalf2(word2);
  float2 scaleZFilter = DecodeHalf2(word3);
  scale = float3(scaleXY.x, scaleXY.y, scaleZFilter.x);
  filterScale = scaleZFilter.y;
  rotation = normalize(float4(DecodeSnorm16(word4), DecodeSnorm16(word4 >> 16u),
                              DecodeSnorm16(word5), DecodeSnorm16(word5 >> 16u)));
}

float3 DecodeSceneGaussianDcColor(uint index) {
  return 0.5f + 0.28209479177387814f * DecodeSceneGaussianRgba(index).rgb;
}

void DecodeSceneGaussianShRest(uint index, out float sh[45]) {
  const uint elementBytes = AttributeBytes(gShFormat);
  [unroll]
  for (uint i = 0u; i < 45u; ++i) {
    sh[i] = LoadPackedAttribute(index, gShOffset + i * elementBytes, gShFormat, true);
  }
}

uint ShCoefficientCount(uint degree) {
  if (degree == 0u) {
    return 1u;
  }
  if (degree == 1u) {
    return 4u;
  }
  if (degree == 2u) {
    return 9u;
  }
  return 16u;
}

float3 EvalSh(float3 baseColor, float sh[45], float3 dir, uint degree) {
  float x = dir.x;
  float y = dir.y;
  float z = dir.z;

  float b[16];
  b[0] = 0.28209479177387814;
  b[1] = -0.4886025119029199 * y;
  b[2] = 0.4886025119029199 * z;
  b[3] = -0.4886025119029199 * x;
  b[4] = 1.0925484305920792 * x * y;
  b[5] = -1.0925484305920792 * y * z;
  b[6] = 0.31539156525252005 * (3.0 * z * z - 1.0);
  b[7] = -1.0925484305920792 * x * z;
  b[8] = 0.5462742152960396 * (x * x - y * y);
  b[9] = -0.5900435899266435 * y * (3.0 * x * x - y * y);
  b[10] = 2.890611442640554 * x * y * z;
  b[11] = -0.4570457994644658 * y * (5.0 * z * z - 1.0);
  b[12] = 0.3731763325901154 * z * (5.0 * z * z - 3.0);
  b[13] = -0.4570457994644658 * x * (5.0 * z * z - 1.0);
  b[14] = 1.445305721320277 * z * (x * x - y * y);
  b[15] = -0.5900435899266435 * x * (x * x - 3.0 * y * y);

  float3 c = baseColor;
  const uint coeffCount = ShCoefficientCount(degree);
  [unroll]
  for (uint i = 1u; i < 16u; ++i) {
    if (i >= coeffCount) {
      break;
    }
    const uint ci = i - 1u;
    c.x += sh[ci] * b[i];
    c.y += sh[15u + ci] * b[i];
    c.z += sh[30u + ci] * b[i];
  }
  return c;
}

float3 ViewVector(float3 v) {
  return float3(
      gView[0][0] * v.x + gView[0][1] * v.y + gView[0][2] * v.z,
      gView[1][0] * v.x + gView[1][1] * v.y + gView[1][2] * v.z,
      gView[2][0] * v.x + gView[2][1] * v.y + gView[2][2] * v.z);
}

float3x3 BuildViewCovariance(float3 scale, float4 q) {
  float xx = q.x * q.x;
  float yy = q.y * q.y;
  float zz = q.z * q.z;
  float xy = q.x * q.y;
  float xz = q.x * q.z;
  float yz = q.y * q.z;
  float xw = q.x * q.w;
  float yw = q.y * q.w;
  float zw = q.z * q.w;

  float3 ex = ViewVector(float3(1.0 - 2.0 * (yy + zz), 2.0 * (xy + zw), 2.0 * (xz - yw)));
  float3 ey = ViewVector(float3(2.0 * (xy - zw), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + xw)));
  float3 ez = ViewVector(float3(2.0 * (xz + yw), 2.0 * (yz - xw), 1.0 - 2.0 * (xx + yy)));

  float sx2 = scale.x * scale.x;
  float sy2 = scale.y * scale.y;
  float sz2 = scale.z * scale.z;

  float3x3 c = 0;
  c += sx2 * float3x3(
      ex.x * ex.x, ex.x * ex.y, ex.x * ex.z,
      ex.y * ex.x, ex.y * ex.y, ex.y * ex.z,
      ex.z * ex.x, ex.z * ex.y, ex.z * ex.z);
  c += sy2 * float3x3(
      ey.x * ey.x, ey.x * ey.y, ey.x * ey.z,
      ey.y * ey.x, ey.y * ey.y, ey.y * ey.z,
      ey.z * ey.x, ey.z * ey.y, ey.z * ey.z);
  c += sz2 * float3x3(
      ez.x * ez.x, ez.x * ez.y, ez.x * ez.z,
      ez.y * ez.x, ez.y * ez.y, ez.y * ez.z,
      ez.z * ez.x, ez.z * ez.y, ez.z * ez.z);
  return c;
}

bool ExtractEllipseAxes(float a, float b, float c, out float2 majorDir, out float2 axisPixels) {
  if (!isfinite(a) || !isfinite(b) || !isfinite(c)) {
    majorDir = float2(1, 0);
    axisPixels = float2(0, 0);
    return false;
  }
  float trace = a + c;
  float diff = a - c;
  float root = sqrt(max(0.0, diff * diff + 4.0 * b * b));
  float eigMajor = max(1e-6, 0.5 * (trace + root));
  float eigMinor = max(1e-6, 0.5 * (trace - root));

  float2 e = float2(b, eigMajor - a);
  if (abs(e.x) + abs(e.y) < 1e-6) {
    e = float2(eigMajor - c, b);
  }
  if (abs(e.x) + abs(e.y) < 1e-6) {
    e = float2(1, 0);
  }
  float len = length(e);
  if (!(len > 0.0) || !isfinite(len)) {
    majorDir = float2(1, 0);
    axisPixels = float2(0, 0);
    return false;
  }
  majorDir = e / len;
  axisPixels = sqrt(float2(eigMajor, eigMinor));
  return isfinite(axisPixels.x) && isfinite(axisPixels.y);
}

float2 QuadCorner(uint vertexId) {
  if (vertexId == 0u) return float2(-1.0f, -1.0f);
  if (vertexId == 1u) return float2(1.0f, -1.0f);
  if (vertexId == 2u) return float2(-1.0f, 1.0f);
  return float2(1.0f, 1.0f);
}

float GaussianPower(float2 positionXY, float2 centerPixel, float4 conicOpacity) {
  const float2 d = positionXY - centerPixel;
  return -0.5f * (conicOpacity.x * d.x * d.x + 2.0f * conicOpacity.y * d.x * d.y + conicOpacity.z * d.y * d.y);
}

bool BuildRasterGaussian(uint sceneIndex, out RasterGaussian outG) {
  outG = (RasterGaussian)0;
  if (sceneIndex >= gSceneCount) {
    return false;
  }

  uint chunkIndex = gSceneIndexToChunk[sceneIndex];
  if (chunkIndex >= gSetCount) {
    return false;
  }

  ChunkPrepGpu chunkPrep = gChunkPrep[chunkIndex];
  if (sceneIndex < chunkPrep.gaussianOffset || sceneIndex >= chunkPrep.gaussianOffset + chunkPrep.gaussianCount) {
    return false;
  }

  SetParamsGpu sp = chunkPrep.params;
  if (sp.visible == 0u) {
    return false;
  }

  float3 position;
  float opacityRaw;
  float3 baseScale;
  float filterScale;
  float4 rotation;
  DecodeSceneGaussianBase(sceneIndex, sp, position, opacityRaw, baseScale, filterScale, rotation);

  float scaleMultiplier = max(sp.scalingModifier * gGlobalScale, 1e-4);
  float3 scaled = max(baseScale * scaleMultiplier, 1e-4);
  scaled = max(scaled, 1e-4);
  float4 posView4 = mul(gView, float4(position, 1.0));
  float viewDepth = posView4.z * (gPositiveViewSpaceZ != 0u ? 1.0f : -1.0f);
  if (viewDepth <= 1e-4f || !isfinite(posView4.x) || !isfinite(posView4.y) || !isfinite(posView4.z) || !isfinite(posView4.w)) {
    return false;
  }

  float4 clip = mul(gProj, posView4);
  if (abs(clip.w) < 1e-6 || !isfinite(clip.x) || !isfinite(clip.y) || !isfinite(clip.z) || !isfinite(clip.w)) {
    return false;
  }

  float3 ndc3 = clip.xyz / clip.w;
  float2 ndc = ndc3.xy;
  if (gFastCulling != 0u) {
    const float dilation = max(gFrustumDilation, 0.0f);
    if (ndc3.x < -1.0f - dilation || ndc3.x > 1.0f + dilation ||
        ndc3.y < -1.0f - dilation || ndc3.y > 1.0f + dilation ||
        ndc3.z < -dilation || ndc3.z > 1.0f) {
      return false;
    }
  }

  float2 centerPixel = float2((ndc.x + 1.0) / gNdcX, (1.0 - ndc.y) / gNdcY);
  const float3 dcColor = DecodeSceneGaussianDcColor(sceneIndex);

  float3 posView = posView4.xyz;
  float3x3 cov = BuildViewCovariance(scaled, rotation);

  float z = max(abs(viewDepth), 1e-4);
  float invZ = 1.0 / z;
  float invZ2 = invZ * invZ;
  float j00 = gFocalX * invZ;
  float j02 = -gFocalX * posView.x * invZ2;
  float j11 = gFocalY * invZ;
  float j12 = -gFocalY * posView.y * invZ2;

  float c00 = cov[0][0];
  float c01 = 0.5 * (cov[0][1] + cov[1][0]);
  float c02 = 0.5 * (cov[0][2] + cov[2][0]);
  float c11 = cov[1][1];
  float c12 = 0.5 * (cov[1][2] + cov[2][1]);
  float c22 = cov[2][2];
  float support = sqrt(8.0f);

  float covScreen00 = j00 * j00 * c00 + 2.0 * j00 * j02 * c02 + j02 * j02 * c22;
  float covScreen01 = j00 * j11 * c01 + j00 * j12 * c02 + j02 * j11 * c12 + j02 * j12 * c22;
  float covScreen11 = j11 * j11 * c11 + 2.0 * j11 * j12 * c12 + j12 * j12 * c22;
  float baseDet = covScreen00 * covScreen11 - covScreen01 * covScreen01;
  if (!(baseDet > 1e-10) || !isfinite(baseDet)) {
    return false;
  }
  float pixelVariance = (gAntialiasingMode != 0u) ? (0.3f * max(gAntialiasingStrength, 0.0f)) : 0.0f;
  float a = covScreen00 + pixelVariance;
  float b = covScreen01;
  float c = covScreen11 + pixelVariance;
  float det = a * c - b * b;
  if (!(det > 1e-10) || !isfinite(det)) {
    return false;
  }
  float invDet = 1.0 / det;
  float3 conic = float3(c * invDet, -b * invDet, a * invDet);

  static const float kAlphaCut = 1.0f / 255.0f;

  float opacity = saturate(opacityRaw);
  if (gAntialiasingMode != 0u) {
    opacity *= sqrt(saturate(baseDet / det));
  }
  if (opacity < kAlphaCut) {
    return false;
  }

  float2 majorDir;
  float2 axisPixels;
  if (!ExtractEllipseAxes(max(a, 1e-6), b, max(c, 1e-6), majorDir, axisPixels)) {
    return false;
  }
  float2 orth = float2(-majorDir.y, majorDir.x);

  axisPixels = axisPixels * support;
  axisPixels = min(axisPixels, float2(gMaxAxisPixels, gMaxAxisPixels));
  axisPixels = max(axisPixels, 0.5f);

  float alphaCutPower = log(kAlphaCut / max(opacity, 1e-6f));
  const float support2 = support * support;
  alphaCutPower = max(alphaCutPower, -0.5f * support2);

  const float2 u = majorDir;
  const float2 v = orth;
  const float invMajorVar = support2 / max(axisPixels.x * axisPixels.x, 1e-8f);
  const float invMinorVar = support2 / max(axisPixels.y * axisPixels.y, 1e-8f);
  conic = float3(
      invMajorVar * u.x * u.x + invMinorVar * v.x * v.x,
      invMajorVar * u.x * u.y + invMinorVar * v.x * v.y,
      invMajorVar * u.y * u.y + invMinorVar * v.y * v.y);

  if (axisPixels.x < 0.55f && axisPixels.y < 0.55f) {
    return false;
  }

  float3 color = float3(1.0, 1.0, 1.0);
  if (gShadingDegree != 0u) {
    float sh[45];
    DecodeSceneGaussianShRest(sceneIndex, sh);
    float3 viewDelta = position - gCameraPos;
    float len2 = dot(viewDelta, viewDelta);
    float3 viewDir = len2 > 1e-10 ? normalize(viewDelta) : float3(0, 0, 1);
    color = saturate(EvalSh(dcColor, sh, viewDir, gShadingDegree));
  } else {
    color = saturate(dcColor);
  }

  outG.clipPos = clip;
  outG.axisU = float2(majorDir.x * axisPixels.x * gNdcX, -majorDir.y * axisPixels.x * gNdcY);
  outG.axisV = float2(orth.x * axisPixels.y * gNdcX, -orth.y * axisPixels.y * gNdcY);
  outG.centerPixel = centerPixel;
  outG.alphaCutPower = alphaCutPower;
  outG.conicOpacity = float4(conic, opacity);
  outG.color = color;
  outG.viewDistance = length(posView4.xyz / max(abs(posView4.w), 1e-6f));
  return true;
}

struct BeautyVSOut {
  float4 position : SV_Position;
  float2 local : TEXCOORD0;
  nointerpolation float3 color : TEXCOORD1;
  nointerpolation float4 conicOpacity : TEXCOORD2;
  nointerpolation float alphaCutPower : TEXCOORD3;
  nointerpolation float ndcDepth : TEXCOORD4;
  nointerpolation float viewDistance : TEXCOORD5;
};

struct DepthVSOut {
  float4 position : SV_Position;
  float2 local : TEXCOORD0;
  nointerpolation float4 conicOpacity : TEXCOORD1;
  nointerpolation float2 centerPixel : TEXCOORD2;
  nointerpolation float ndcDepth : TEXCOORD3;
  nointerpolation float alphaCutPower : TEXCOORD4;
};

BeautyVSOut VSMainBeauty(uint vertexId : SV_VertexID, uint instanceId : SV_InstanceID) {
  BeautyVSOut o = (BeautyVSOut)0;
  const uint sceneIndex = gSortedSceneIndices[instanceId];
  RasterGaussian g;
  if (!BuildRasterGaussian(sceneIndex, g)) {
    o.position = float4(0, 0, 0, 0);
    return o;
  }
  const float2 q = QuadCorner(vertexId);
  const float2 ndcCenter = g.clipPos.xy / max(g.clipPos.w, 1e-6);
  const float2 ndc = ndcCenter + g.axisU * q.x + g.axisV * q.y;
  o.position = float4(ndc * g.clipPos.w, g.clipPos.z, g.clipPos.w);
  o.local = q;
  o.color = g.color;
  o.conicOpacity = g.conicOpacity;
  o.alphaCutPower = g.alphaCutPower;
  o.ndcDepth = saturate(g.clipPos.z / max(g.clipPos.w, 1e-6));
  o.viewDistance = g.viewDistance;
  return o;
}

DepthVSOut VSMainDepth(uint vertexId : SV_VertexID, uint instanceId : SV_InstanceID) {
  DepthVSOut o = (DepthVSOut)0;
  const uint sceneIndex = gSortedSceneIndices[instanceId];
  RasterGaussian g;
  if (!BuildRasterGaussian(sceneIndex, g)) {
    o.position = float4(0, 0, 0, 0);
    return o;
  }
  const float2 q = QuadCorner(vertexId);
  const float2 ndcCenter = g.clipPos.xy / max(g.clipPos.w, 1e-6);
  const float2 ndc = ndcCenter + g.axisU * q.x + g.axisV * q.y;
  o.position = float4(ndc * g.clipPos.w, g.clipPos.z, g.clipPos.w);
  o.local = q;
  o.conicOpacity = g.conicOpacity;
  o.centerPixel = g.centerPixel;
  o.ndcDepth = saturate(g.clipPos.z / max(g.clipPos.w, 1e-6));
  o.alphaCutPower = g.alphaCutPower;
  return o;
}

float4 PSMainBeauty(BeautyVSOut i) : SV_Target {
  const float r2 = dot(i.local, i.local);
  if (r2 > 1.0f) {
    discard;
  }

  const float power = -4.0f * r2;
  if (power < i.alphaCutPower) {
    discard;
  }

  float alpha = min(0.99f, saturate(i.conicOpacity.w) * exp(power));
  if (alpha < 1.0f / 255.0f) {
    discard;
  }

  if (gRenderType == 1u) {
    return float4(1.0f, 1.0f, 1.0f, alpha);
  }
  if (gRenderType == 2u) {
    const float depth = 1.0f - saturate(i.viewDistance / 10.0f);
    return float4(depth, depth, depth, alpha);
  }

  float3 color = i.color;
  if (gGammaCorrection != 0u) {
    color = pow(saturate(color), 1.0f / 2.2f);
  }
  return float4(color, alpha);
}

float PSDepth(DepthVSOut i) : SV_Depth {
  const float r2 = dot(i.local, i.local);
  if (r2 > 1.0f) {
    discard;
  }
  const float power = GaussianPower(i.position.xy, i.centerPixel, i.conicOpacity);
  if (power > 0.0f || power < i.alphaCutPower) {
    discard;
  }
  return i.ndcDepth;
}
