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
StructuredBuffer<ChunkPrepGpu> gChunkPrep : register(t1);
RWStructuredBuffer<uint> gOutSortKeys : register(u0);
RWStructuredBuffer<uint> gOutSortValues : register(u1);
RWByteAddressBuffer gVisibleCounter : register(u2);

static const uint kSplatAlphaHistogramBins = 50u;
static const uint kSplatAlphaHistogramOffset = 8u;
static const uint kProjectionThreadHistogramOffset = kSplatAlphaHistogramOffset + kSplatAlphaHistogramBins * 4u;
static const uint kProjectionThreadHistogramBins = 64u;
static const uint kProjectionSubgroupSize = 32u;
static const uint kProjectionSubgroupCount = 8u;
static const uint kFormatFloat32 = 0u;
static const uint kFormatFloat16 = 1u;
static const uint kFormatUint8 = 2u;
static const float kPackedShUint8Range = 4.0f;

groupshared uint gPrepareSubgroupActiveThreads[8];

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

uint EncodeDepthKey(float depth) {
  uint bits = asuint(depth);
  uint mask = ((bits & 0x80000000u) != 0u) ? 0xFFFFFFFFu : 0x80000000u;
  uint sortableAscending = bits ^ mask;
  return ~sortableAscending;
}

[numthreads(256, 1, 1)]
void CSPrepare(uint3 groupId : SV_GroupID, uint3 groupThreadId : SV_GroupThreadID) {
  if (groupThreadId.x < kProjectionSubgroupCount) {
    gPrepareSubgroupActiveThreads[groupThreadId.x] = 0u;
  }
  GroupMemoryBarrierWithGroupSync();

  bool active = false;
  uint idx = 0u;
  float viewDepth = 0.0f;
  float projectedAlpha = 0.0f;
  uint chunkIndex = groupId.y;
  if (chunkIndex < gSetCount) {
    ChunkPrepGpu chunkPrep = gChunkPrep[chunkIndex];
    SetParamsGpu sp = chunkPrep.params;
    uint localIndex = groupId.x * 256u + groupThreadId.x;
    if (sp.visible != 0u && localIndex < chunkPrep.gaussianCount) {
      idx = chunkPrep.gaussianOffset + localIndex;
      if (idx < gSceneCount) {
        float3 position;
        float opacityRaw;
        float3 baseScale;
        float filterScale;
        float4 rotation;
        DecodeSceneGaussianBase(idx, sp, position, opacityRaw, baseScale, filterScale, rotation);

        float scaleMultiplier = max(sp.scalingModifier * gGlobalScale, 1e-4);
        float3 scaled = max(baseScale * scaleMultiplier, 1e-4);
        scaled = max(scaled, 1e-4);
        float4 posView4 = mul(gView, float4(position, 1.0));
        viewDepth = posView4.z * (gPositiveViewSpaceZ != 0u ? 1.0f : -1.0f);
        bool valid = viewDepth > 1e-4f && isfinite(posView4.x) && isfinite(posView4.y) &&
                     isfinite(posView4.z) && isfinite(posView4.w);

        float4 clip = 0.0f;
        if (valid) {
          clip = mul(gProj, posView4);
          valid = abs(clip.w) >= 1e-6 && isfinite(clip.x) && isfinite(clip.y) && isfinite(clip.z) && isfinite(clip.w);
        }

        float3 ndc3 = 0.0f;
        float2 ndc = 0.0f;
        if (valid) {
          ndc3 = clip.xyz / clip.w;
          ndc = ndc3.xy;
          if (gFastCulling != 0u) {
            const float dilation = max(gFrustumDilation, 0.0f);
            valid = !(ndc3.x < -1.0f - dilation || ndc3.x > 1.0f + dilation ||
                      ndc3.y < -1.0f - dilation || ndc3.y > 1.0f + dilation ||
                      ndc3.z < -dilation || ndc3.z > 1.0f);
          }
        }

        float3 posView = posView4.xyz;
        float3x3 cov = BuildViewCovariance(scaled, rotation);

        float a = 0.0f;
        float b = 0.0f;
        float c = 0.0f;
        float baseDet = 0.0f;
        float det = 0.0f;
        float opacity = 0.0f;
        if (valid) {
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

          float covScreen00 = j00 * j00 * c00 + 2.0 * j00 * j02 * c02 + j02 * j02 * c22;
          float covScreen01 = j00 * j11 * c01 + j00 * j12 * c02 + j02 * j11 * c12 + j02 * j12 * c22;
          float covScreen11 = j11 * j11 * c11 + 2.0 * j11 * j12 * c12 + j12 * j12 * c22;
          baseDet = covScreen00 * covScreen11 - covScreen01 * covScreen01;
          valid = baseDet > 1e-10 && isfinite(baseDet);

          float pixelVariance = (gAntialiasingMode != 0u) ? (0.3f * max(gAntialiasingStrength, 0.0f)) : 0.0f;
          a = covScreen00 + pixelVariance;
          b = covScreen01;
          c = covScreen11 + pixelVariance;
          det = a * c - b * b;
          valid = valid && det > 1e-10 && isfinite(det);
          opacity = saturate(opacityRaw);
          if (gAntialiasingMode != 0u && det > 1e-10) {
            opacity *= sqrt(saturate(baseDet / det));
          }
        }

        static const float kAlphaCut = 1.0f / 255.0f;

        float2 majorDir = 0.0f;
        float2 axisPixels = 0.0f;
        if (valid) {
          valid = opacity >= kAlphaCut && ExtractEllipseAxes(max(a, 1e-6), b, max(c, 1e-6), majorDir, axisPixels);
        }

        if (valid) {
          float support = sqrt(8.0f);
          axisPixels = axisPixels * support;
          axisPixels = min(axisPixels, float2(gMaxAxisPixels, gMaxAxisPixels));
          axisPixels = max(axisPixels, 0.5f);
          valid = !(axisPixels.x < 0.55f && axisPixels.y < 0.55f);
        }

        active = valid;
        projectedAlpha = opacity;
      }
    }
  }

  if (active) {
    uint ignored;
    uint alphaBin = min((uint)(saturate(projectedAlpha) * (float)kSplatAlphaHistogramBins), kSplatAlphaHistogramBins - 1u);
    gVisibleCounter.InterlockedAdd(kSplatAlphaHistogramOffset + alphaBin * 4u, 1u, ignored);
    InterlockedAdd(gPrepareSubgroupActiveThreads[groupThreadId.x / kProjectionSubgroupSize], 1u, ignored);
  }
  GroupMemoryBarrierWithGroupSync();
  if (groupThreadId.x < kProjectionSubgroupCount) {
    uint count = gPrepareSubgroupActiveThreads[groupThreadId.x];
    if (count > 0u) {
      uint bin = min(count - 1u, kProjectionThreadHistogramBins - 1u);
      uint ignored;
      gVisibleCounter.InterlockedAdd(kProjectionThreadHistogramOffset + bin * 4u, 1u, ignored);
    }
  }

  if (!active) {
    return;
  }

  uint pairIndex;
  gVisibleCounter.InterlockedAdd(0, 1, pairIndex);
  if (pairIndex >= gPairCapacity) {
    return;
  }

  uint sortKey = EncodeDepthKey(viewDepth);
  gOutSortKeys[pairIndex] = sortKey;
  gOutSortValues[pairIndex] = idx;

  uint visibleIndex;
  gVisibleCounter.InterlockedAdd(4, 1, visibleIndex);
}

struct SortMetaGpu {
  uint pairCount;
  uint visibleCount;
  uint visibleBlocks;
  uint sortPassCount;
  uint sortCount;
  uint oneSweepPartitions;
  uint oneSweepGlobalHistPartitions;
  uint packDispatchCount;
};

cbuffer SortMetaConstants : register(b0) {
  uint gSortCapacity;
  uint gSortGroupSize;
  uint gSortPassCount;
  uint gOneSweepPartitionSize;
  uint gOneSweepGlobalHistPartitionSize;
  uint gIndirectCommandStride;
  uint gPackGroupSize;
  uint gSortPad;
};

RWStructuredBuffer<uint> gSortKeysFill : register(u0);
RWStructuredBuffer<uint> gSortValuesFill : register(u1);
RWByteAddressBuffer gSortVisibleCounter : register(u2);

[numthreads(256, 1, 1)]
void CSFillSortTail(uint3 tid : SV_DispatchThreadID) {
  uint index = tid.x;
  if (index >= gSortCapacity) {
    return;
  }
  uint visibleCount = min(gSortVisibleCounter.Load(0), gSortCapacity);
  if (index < visibleCount) {
    return;
  }
  gSortKeysFill[index] = 0xFFFFFFFFu;
  gSortValuesFill[index] = 0u;
}

RWByteAddressBuffer gSortMetaVisibleCounter : register(u0);
RWByteAddressBuffer gSortMetaOut : register(u1);
RWStructuredBuffer<uint> gSortMetaKeys : register(u2);
RWStructuredBuffer<uint> gSortMetaValues : register(u3);

[numthreads(1, 1, 1)]
void CSBuildSortMeta(uint3 tid : SV_DispatchThreadID) {
  uint pairCount = min(gSortMetaVisibleCounter.Load(0), gSortCapacity);
  uint visibleCount = gSortMetaVisibleCounter.Load(4);
  uint sortCount = max(pairCount, 1u);
  uint visibleBlocks = (sortCount + (gSortGroupSize - 1u)) / max(gSortGroupSize, 1u);
  uint oneSweepPartitions = (sortCount + (max(gOneSweepPartitionSize, 1u) - 1u)) / max(gOneSweepPartitionSize, 1u);
  uint oneSweepGlobalHistPartitions = (sortCount + (max(gOneSweepGlobalHistPartitionSize, 1u) - 1u)) / max(gOneSweepGlobalHistPartitionSize, 1u);
  uint packDispatchCount = (sortCount + (max(gPackGroupSize, 1u) - 1u)) / max(gPackGroupSize, 1u);
  if (pairCount == 0u) {
    gSortMetaKeys[0] = 0xFFFFFFFFu;
    gSortMetaValues[0] = 0u;
  }
  gSortMetaOut.Store(0, pairCount);
  gSortMetaOut.Store(4, visibleCount);
  gSortMetaOut.Store(8, visibleBlocks);
  gSortMetaOut.Store(12, gSortPassCount);
  gSortMetaOut.Store(16, sortCount);
  gSortMetaOut.Store(20, oneSweepPartitions);
  gSortMetaOut.Store(24, oneSweepGlobalHistPartitions);
  gSortMetaOut.Store(28, packDispatchCount);
}

RWByteAddressBuffer gDispatchArgsSortMeta : register(u0);
RWByteAddressBuffer gDispatchArgsOut : register(u1);

void StoreIndirectDispatchCommand(uint byteOffset, uint4 constants, uint3 dispatch) {
  gDispatchArgsOut.Store(byteOffset + 0u, constants.x);
  gDispatchArgsOut.Store(byteOffset + 4u, constants.y);
  gDispatchArgsOut.Store(byteOffset + 8u, constants.z);
  gDispatchArgsOut.Store(byteOffset + 12u, constants.w);
  gDispatchArgsOut.Store(byteOffset + 16u, dispatch.x);
  gDispatchArgsOut.Store(byteOffset + 20u, dispatch.y);
  gDispatchArgsOut.Store(byteOffset + 24u, dispatch.z);
}

[numthreads(1, 1, 1)]
void CSBuildOneSweepDispatchArgs(uint3 tid : SV_DispatchThreadID) {
  const uint pairCount = gDispatchArgsSortMeta.Load(16);
  const uint partitions = gDispatchArgsSortMeta.Load(20);
  const uint globalHistPartitions = gDispatchArgsSortMeta.Load(24);
  const uint stride = 28u;

  StoreIndirectDispatchCommand(stride * 0u, uint4(0u, 0u, partitions, 0u), uint3(256u, 1u, 1u));
  StoreIndirectDispatchCommand(stride * 1u, uint4(pairCount, 0u, globalHistPartitions, 0u), uint3(globalHistPartitions, 1u, 1u));
  StoreIndirectDispatchCommand(stride * 2u, uint4(0u, 0u, partitions, 0u), uint3(4u, 1u, 1u));
  StoreIndirectDispatchCommand(stride * 3u, uint4(pairCount, 0u, partitions, 0u), uint3(partitions, 1u, 1u));
  StoreIndirectDispatchCommand(stride * 4u, uint4(pairCount, 8u, partitions, 0u), uint3(partitions, 1u, 1u));
  StoreIndirectDispatchCommand(stride * 5u, uint4(pairCount, 16u, partitions, 0u), uint3(partitions, 1u, 1u));
  StoreIndirectDispatchCommand(stride * 6u, uint4(pairCount, 24u, partitions, 0u), uint3(partitions, 1u, 1u));
}

RWByteAddressBuffer gFinalizeVisibleCounter : register(u0);
RWByteAddressBuffer gDrawArgs : register(u1);
RWByteAddressBuffer gFinalizeSortMeta : register(u2);
[numthreads(1, 1, 1)]
void CSReset(uint3 tid : SV_DispatchThreadID) {
  gFinalizeVisibleCounter.Store(0, 0);
  gFinalizeVisibleCounter.Store(4, 0);
  [unroll]
  for (uint alphaBin = 0u; alphaBin < kSplatAlphaHistogramBins; ++alphaBin) {
    gFinalizeVisibleCounter.Store(kSplatAlphaHistogramOffset + alphaBin * 4u, 0);
  }
  [unroll]
  for (uint i = 0u; i < kProjectionThreadHistogramBins; ++i) {
    gFinalizeVisibleCounter.Store(kProjectionThreadHistogramOffset + i * 4u, 0);
  }
  gDrawArgs.Store(0, 0);
  gDrawArgs.Store(4, 0);
  gDrawArgs.Store(8, 0);
  gDrawArgs.Store(12, 0);
}

[numthreads(1, 1, 1)]
void CSFinalizeDrawArgs(uint3 tid : SV_DispatchThreadID) {
  uint pairCount = gFinalizeSortMeta.Load(0);
  gDrawArgs.Store(0, 4u);
  gDrawArgs.Store(4, pairCount);
  gDrawArgs.Store(8, 0u);
  gDrawArgs.Store(12, 0u);
}
