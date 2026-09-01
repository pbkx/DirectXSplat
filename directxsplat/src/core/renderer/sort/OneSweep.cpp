#include "renderer/sort/OneSweep.h"

#include "EmbeddedShaders.h"

#include <Windows.h>
#include <d3dx12.h>
#include <dxcapi.h>
#include <dxgi1_6.h>
#include <wrl/implements.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <string>
#include <string_view>

#include "GPUSorting/GPUSortingD3D12/Tuner.h"

namespace directxsplat {

namespace {

using Microsoft::WRL::ComPtr;

constexpr uint32_t kRadix = 256u;
constexpr uint32_t kRadixPasses = 4u;
constexpr uint32_t kGlobalHistPartitionSize = 32768u;
constexpr uint32_t kMaxDispatchDimension = 65535u;
constexpr uint32_t kShaderModel60Value = 0x60u;
constexpr std::array<D3D_SHADER_MODEL, 8> kRequestedShaderModels = {
    static_cast<D3D_SHADER_MODEL>(0x67),
    static_cast<D3D_SHADER_MODEL>(0x66),
    static_cast<D3D_SHADER_MODEL>(0x65),
    static_cast<D3D_SHADER_MODEL>(0x64),
    static_cast<D3D_SHADER_MODEL>(0x63),
    static_cast<D3D_SHADER_MODEL>(0x62),
    static_cast<D3D_SHADER_MODEL>(0x61),
    static_cast<D3D_SHADER_MODEL>(kShaderModel60Value),
};

struct EmbeddedDxcSource {
  const char* name = nullptr;
  const char* source = nullptr;
  size_t size = 0;
};

std::wstring_view Basename(std::wstring_view path) {
  const size_t slash = path.find_last_of(L"/\\");
  return slash == std::wstring_view::npos ? path : path.substr(slash + 1u);
}

const EmbeddedDxcSource* FindOneSweepSource(std::wstring_view filename) {
  const std::wstring_view filenameBase = Basename(filename);
  static constexpr EmbeddedDxcSource kSources[] = {
      {embedded::kOneSweepShaderName, embedded::kOneSweepShaderSource, embedded::kOneSweepShaderSize},
      {embedded::kSweepCommonShaderName, embedded::kSweepCommonShaderSource, embedded::kSweepCommonShaderSize},
      {embedded::kSortCommonShaderName, embedded::kSortCommonShaderSource, embedded::kSortCommonShaderSize},
  };
  for (const EmbeddedDxcSource& source : kSources) {
    std::wstring fullName(source.name, source.name + std::strlen(source.name));
    const std::wstring_view fullNameView(fullName);
    const std::wstring_view fullBase = Basename(fullNameView);
    if (filename == fullNameView || filenameBase == fullBase) {
      return &source;
    }
    if (filename.find(fullBase) != std::wstring_view::npos) {
      return &source;
    }
  }
  return nullptr;
}

class EmbeddedIncludeHandler final
    : public Microsoft::WRL::RuntimeClass<Microsoft::WRL::RuntimeClassFlags<Microsoft::WRL::ClassicCom>,
                                          IDxcIncludeHandler> {
 public:
  explicit EmbeddedIncludeHandler(IDxcUtils* utils) : utils_(utils) {}

  HRESULT STDMETHODCALLTYPE LoadSource(LPCWSTR filename, IDxcBlob** includeSource) override {
    if (includeSource == nullptr) {
      return E_POINTER;
    }
    *includeSource = nullptr;
    if (filename == nullptr) {
      return E_INVALIDARG;
    }
    const EmbeddedDxcSource* source = FindOneSweepSource(filename);
    if (source == nullptr) {
      return HRESULT_FROM_WIN32(ERROR_FILE_NOT_FOUND);
    }
    ComPtr<IDxcBlobEncoding> blob;
    HRESULT hr = utils_->CreateBlob(source->source, static_cast<UINT32>(source->size), DXC_CP_UTF8, blob.GetAddressOf());
    if (FAILED(hr)) {
      return hr;
    }
    *includeSource = blob.Detach();
    return S_OK;
  }

 private:
  ComPtr<IDxcUtils> utils_;
};

void UavBarrier(ID3D12GraphicsCommandList* commandList, ID3D12Resource* resource) {
  D3D12_RESOURCE_BARRIER barrier{};
  barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_UAV;
  barrier.UAV.pResource = resource;
  commandList->ResourceBarrier(1, &barrier);
}

uint32_t DivRoundUp(uint32_t value, uint32_t divisor) {
  return divisor == 0 ? 0u : (value + divisor - 1u) / divisor;
}

}  // namespace

struct OneSweep::Kernel {
  ComPtr<ID3D12RootSignature> rootSignature;
  ComPtr<ID3D12PipelineState> pipelineState;
  ComPtr<ID3D12CommandSignature> indirectCommandSignature;
};

OneSweep::OneSweep() = default;

OneSweep::~OneSweep() = default;

bool OneSweep::IsInitialized() const {
  return initSweepKernel_ != nullptr && globalHistogramKernel_ != nullptr && scanKernel_ != nullptr &&
         digitBinningKernel_ != nullptr;
}

uint32_t OneSweep::MaxPartitionsForElementCount(uint32_t elementCount) const {
  const uint32_t partitionSize = std::max<uint32_t>(tuning_.partitionSize, 1u);
  return DivRoundUp(std::max<uint32_t>(elementCount, 1u), partitionSize);
}

uint32_t OneSweep::PartitionSize() const {
  return std::max<uint32_t>(tuning_.partitionSize, 1u);
}

Status OneSweep::Initialize(ID3D12Device* device) try {
  Shutdown();
  if (device == nullptr) {
    return Status::Error("invalid OneSweep device");
  }
  device_ = device;
  Status s = QueryDeviceInfo();
  if (!s.ok) {
    Shutdown();
    return s;
  }
  s = BuildCompileArguments();
  if (!s.ok) {
    Shutdown();
    return s;
  }
  s = InitializeKernels();
  if (!s.ok) {
    Shutdown();
    return s;
  }
  return Status::Ok();
} catch (const std::bad_alloc&) {
  Shutdown();
  return Status::Error("OneSweep allocation failed");
} catch (const std::length_error&) {
  Shutdown();
  return Status::Error("OneSweep allocation failed");
} catch (const std::exception&) {
  Shutdown();
  return Status::Error("OneSweep initialization failed");
} catch (...) {
  Shutdown();
  return Status::Error("OneSweep initialization failed");
}

void OneSweep::Shutdown() {
  initSweepKernel_.reset();
  globalHistogramKernel_.reset();
  scanKernel_.reset();
  digitBinningKernel_.reset();
  compileArguments_.clear();
  tuning_ = {};
  deviceInfo_ = {};
  device_ = nullptr;
}

Status OneSweep::QueryDeviceInfo() {
  if (device_ == nullptr) {
    return Status::Error("invalid OneSweep device");
  }

  deviceInfo_ = {};
  auto adapterLuid = device_->GetAdapterLuid();
  ComPtr<IDXGIFactory4> factory;
  HRESULT hr = CreateDXGIFactory2(0, IID_PPV_ARGS(factory.GetAddressOf()));
  if (FAILED(hr)) {
    return Status::Error("failed creating DXGI factory for OneSweep");
  }

  ComPtr<IDXGIAdapter1> adapter;
  hr = factory->EnumAdapterByLuid(adapterLuid, IID_PPV_ARGS(adapter.GetAddressOf()));
  if (FAILED(hr)) {
    return Status::Error("failed resolving OneSweep adapter");
  }

  DXGI_ADAPTER_DESC1 adapterDesc{};
  adapter->GetDesc1(&adapterDesc);
  deviceInfo_.Description = adapterDesc.Description;
  deviceInfo_.deviceId = adapterDesc.DeviceId;
  deviceInfo_.vendorId = adapterDesc.VendorId;
  deviceInfo_.dedicatedVideoMemory = adapterDesc.DedicatedVideoMemory;
  deviceInfo_.sharedSystemMemory = adapterDesc.SharedSystemMemory;

  const bool isWarpDevice = ((adapterDesc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) != 0) ||
                            (_wcsicmp(adapterDesc.Description, L"Microsoft Basic Render Driver") == 0);

  D3D12_FEATURE_DATA_SHADER_MODEL model{};
  // Numeric values compile with SDKs that predate SM 6.7. Older runtimes require
  // retrying lower models when they reject an unknown maximum with E_INVALIDARG.
  for (const D3D_SHADER_MODEL requestedModel : kRequestedShaderModels) {
    model.HighestShaderModel = requestedModel;
    hr = device_->CheckFeatureSupport(D3D12_FEATURE_SHADER_MODEL, &model, sizeof(model));
    if (hr != E_INVALIDARG) {
      break;
    }
  }
  if (FAILED(hr) || static_cast<uint32_t>(model.HighestShaderModel) < kShaderModel60Value) {
    return Status::Error("OneSweep requires shader model 6.0 or newer");
  }

  static constexpr const wchar_t* kShaderModelNames[] = {L"cs_6_0", L"cs_6_1", L"cs_6_2", L"cs_6_3",
                                                           L"cs_6_4", L"cs_6_5", L"cs_6_6", L"cs_6_7"};
  const uint32_t shaderModelIndex =
      static_cast<uint32_t>(model.HighestShaderModel) - kShaderModel60Value;
  if (shaderModelIndex >= _countof(kShaderModelNames)) {
    return Status::Error("unsupported shader model for OneSweep");
  }
  deviceInfo_.SupportedShaderModel = kShaderModelNames[shaderModelIndex];

  D3D12_FEATURE_DATA_D3D12_OPTIONS1 options1{};
  hr = device_->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS1, &options1, sizeof(options1));
  if (FAILED(hr)) {
    return Status::Error("failed querying D3D12 wave features for OneSweep");
  }

  D3D12_FEATURE_DATA_D3D12_OPTIONS4 options4{};
  hr = device_->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS4, &options4, sizeof(options4));
  if (FAILED(hr)) {
    return Status::Error("failed querying D3D12 16-bit features for OneSweep");
  }

  deviceInfo_.SIMDWidth = options1.WaveLaneCountMin;
  deviceInfo_.SIMDMaxWidth = options1.WaveLaneCountMax;
  deviceInfo_.SIMDLaneCount = options1.TotalLaneCount;
  deviceInfo_.SupportsWaveIntrinsics = options1.WaveOps;
  deviceInfo_.Supports16BitTypes = options4.Native16BitShaderOpsSupported;
  deviceInfo_.SupportsDeviceRadixSort = deviceInfo_.SIMDWidth >= 4 && deviceInfo_.SupportsWaveIntrinsics;
  deviceInfo_.SupportsOneSweep = deviceInfo_.SupportsDeviceRadixSort && !isWarpDevice;
  if (!deviceInfo_.SupportsOneSweep) {
    return Status::Error("OneSweep requires hardware wave-op support and does not run on WARP");
  }

  tuning_ = Tuner::GetTuningParameters(deviceInfo_, GPUSorting::MODE_PAIRS);
  return Status::Ok();
}

Status OneSweep::BuildCompileArguments() {
  compileArguments_.clear();

  if (tuning_.shouldLockWavesTo32) {
    compileArguments_.push_back(L"-DLOCK_TO_W32");
  }

  switch (tuning_.keysPerThread) {
    case 5:
      compileArguments_.push_back(L"-DKEYS_PER_THREAD_5");
      compileArguments_.push_back(L"-DKEYS_PER_THREAD_7");
      break;
    case 7:
      compileArguments_.push_back(L"-DKEYS_PER_THREAD_7");
      break;
    case 15:
      break;
    default:
      return Status::Error("unsupported OneSweep tuning: keysPerThread");
  }

  switch (tuning_.threadsPerThreadblock) {
    case 256:
      compileArguments_.push_back(L"-DD_DIM_256");
      break;
    case 512:
      break;
    default:
      return Status::Error("unsupported OneSweep tuning: threadsPerThreadblock");
  }

  switch (tuning_.partitionSize) {
    case 1792:
      compileArguments_.push_back(L"-DPART_SIZE_1792");
      break;
    case 2560:
      compileArguments_.push_back(L"-DPART_SIZE_2560");
      break;
    case 3584:
      compileArguments_.push_back(L"-DPART_SIZE_3584");
      break;
    case 3840:
      compileArguments_.push_back(L"-DPART_SIZE_3840");
      break;
    case 7680:
      break;
    default:
      return Status::Error("unsupported OneSweep tuning: partitionSize");
  }

  switch (tuning_.totalSharedMemory) {
    case 4096:
      compileArguments_.push_back(L"-DD_TOTAL_SMEM_4096");
      break;
    case 7936:
      break;
    default:
      return Status::Error("unsupported OneSweep tuning: totalSharedMemory");
  }

  compileArguments_.push_back(L"-DSHOULD_ASCEND");
  compileArguments_.push_back(L"-DKEY_UINT");
  compileArguments_.push_back(L"-DSORT_PAIRS");
  compileArguments_.push_back(L"-DPAYLOAD_UINT");
  if (deviceInfo_.Supports16BitTypes) {
    compileArguments_.push_back(L"-enable-16bit-types");
    compileArguments_.push_back(L"-DENABLE_16_BIT");
  }
  compileArguments_.push_back(L"-O3");
  return Status::Ok();
}

Status OneSweep::CompileKernel(const wchar_t* entryPoint,
                               const std::vector<CD3DX12_ROOT_PARAMETER1>& rootParameters,
                               std::unique_ptr<Kernel>& outKernel) {
  outKernel.reset();
  ComPtr<IDxcUtils> utils;
  HRESULT hr = DxcCreateInstance(CLSID_DxcUtils, IID_PPV_ARGS(utils.GetAddressOf()));
  if (FAILED(hr)) {
    return Status::Error("failed creating DXC utils for OneSweep");
  }

  ComPtr<IDxcCompiler3> compiler;
  hr = DxcCreateInstance(CLSID_DxcCompiler, IID_PPV_ARGS(compiler.GetAddressOf()));
  if (FAILED(hr)) {
    return Status::Error("failed creating DXC compiler for OneSweep");
  }

  ComPtr<IDxcBlobEncoding> sourceBlob;
  hr = utils->CreateBlob(embedded::kOneSweepShaderSource, static_cast<UINT32>(embedded::kOneSweepShaderSize), DXC_CP_UTF8,
                         sourceBlob.GetAddressOf());
  if (FAILED(hr)) {
    return Status::Error("failed creating OneSweep source blob");
  }

  std::vector<LPCWSTR> arguments;
  std::wstring sourceNameW(embedded::kOneSweepShaderName, embedded::kOneSweepShaderName + std::strlen(embedded::kOneSweepShaderName));
  arguments.push_back(sourceNameW.c_str());
  arguments.push_back(L"-E");
  arguments.push_back(entryPoint);
  arguments.push_back(L"-T");
  arguments.push_back(deviceInfo_.SupportedShaderModel.c_str());
  arguments.push_back(L"-Zpr");
  for (const std::wstring& argument : compileArguments_) {
    arguments.push_back(argument.c_str());
  }

  DxcBuffer buffer{};
  buffer.Ptr = sourceBlob->GetBufferPointer();
  buffer.Size = sourceBlob->GetBufferSize();
  buffer.Encoding = DXC_CP_UTF8;

  ComPtr<EmbeddedIncludeHandler> includeHandler = Microsoft::WRL::Make<EmbeddedIncludeHandler>(utils.Get());

  ComPtr<IDxcResult> result;
  hr = compiler->Compile(&buffer, arguments.data(), static_cast<UINT32>(arguments.size()), includeHandler.Get(),
                         IID_PPV_ARGS(result.GetAddressOf()));
  if (FAILED(hr)) {
    return Status::Error("failed compiling OneSweep shader");
  }

  HRESULT status = S_OK;
  result->GetStatus(&status);
  if (FAILED(status)) {
    ComPtr<IDxcBlobUtf8> errors;
    std::string message = "OneSweep shader compile failed";
    if (SUCCEEDED(result->GetOutput(DXC_OUT_ERRORS, IID_PPV_ARGS(errors.GetAddressOf()), nullptr)) && errors != nullptr &&
        errors->GetStringLength() > 0) {
      message += ": ";
      message += errors->GetStringPointer();
    }
    return Status::Error(std::move(message));
  }

  ComPtr<IDxcBlob> byteCode;
  hr = result->GetOutput(DXC_OUT_OBJECT, IID_PPV_ARGS(byteCode.GetAddressOf()), nullptr);
  if (FAILED(hr)) {
    return Status::Error("failed extracting OneSweep shader bytecode");
  }

  CD3DX12_VERSIONED_ROOT_SIGNATURE_DESC signatureDesc;
  signatureDesc.Init_1_1(static_cast<UINT>(rootParameters.size()), rootParameters.data(), 0, nullptr);

  ComPtr<ID3DBlob> signatureBlob;
  ComPtr<ID3DBlob> signatureErrors;
  hr = D3DX12SerializeVersionedRootSignature(&signatureDesc, D3D_ROOT_SIGNATURE_VERSION_1_1,
                                             signatureBlob.GetAddressOf(), signatureErrors.GetAddressOf());
  if (FAILED(hr)) {
    return Status::Error("failed serializing OneSweep root signature");
  }

  auto kernel = std::make_unique<Kernel>();
  hr = device_->CreateRootSignature(0, signatureBlob->GetBufferPointer(), signatureBlob->GetBufferSize(),
                                    IID_PPV_ARGS(kernel->rootSignature.GetAddressOf()));
  if (FAILED(hr)) {
    return Status::Error("failed creating OneSweep root signature");
  }

  D3D12_COMPUTE_PIPELINE_STATE_DESC pipelineDesc{};
  pipelineDesc.pRootSignature = kernel->rootSignature.Get();
  pipelineDesc.CS.pShaderBytecode = byteCode->GetBufferPointer();
  pipelineDesc.CS.BytecodeLength = byteCode->GetBufferSize();
  hr = device_->CreateComputePipelineState(&pipelineDesc, IID_PPV_ARGS(kernel->pipelineState.GetAddressOf()));
  if (FAILED(hr)) {
    return Status::Error("failed creating OneSweep compute pipeline state");
  }

  D3D12_INDIRECT_ARGUMENT_DESC indirectArgs[2]{};
  indirectArgs[0].Type = D3D12_INDIRECT_ARGUMENT_TYPE_CONSTANT;
  indirectArgs[0].Constant.RootParameterIndex = 0;
  indirectArgs[0].Constant.Num32BitValuesToSet = 4;
  indirectArgs[0].Constant.DestOffsetIn32BitValues = 0;
  indirectArgs[1].Type = D3D12_INDIRECT_ARGUMENT_TYPE_DISPATCH;
  D3D12_COMMAND_SIGNATURE_DESC indirectDesc{};
  indirectDesc.ByteStride = sizeof(uint32_t) * 7u;
  indirectDesc.NumArgumentDescs = 2;
  indirectDesc.pArgumentDescs = indirectArgs;
  hr = device_->CreateCommandSignature(&indirectDesc, kernel->rootSignature.Get(),
                                       IID_PPV_ARGS(kernel->indirectCommandSignature.GetAddressOf()));
  if (FAILED(hr)) {
    return Status::Error("failed creating OneSweep indirect command signature");
  }

  outKernel = std::move(kernel);
  return Status::Ok();
}

Status OneSweep::InitializeKernels() {
  Status s;
  {
    std::vector<CD3DX12_ROOT_PARAMETER1> params(4);
    params[0].InitAsConstants(4, 0);
    params[1].InitAsUnorderedAccessView(4);
    params[2].InitAsUnorderedAccessView(5);
    params[3].InitAsUnorderedAccessView(6);
    s = CompileKernel(L"InitSweep", params, initSweepKernel_);
  }
  if (!s.ok) return s;
  {
    std::vector<CD3DX12_ROOT_PARAMETER1> params(3);
    params[0].InitAsConstants(4, 0);
    params[1].InitAsUnorderedAccessView(0);
    params[2].InitAsUnorderedAccessView(4);
    s = CompileKernel(L"GlobalHistogram", params, globalHistogramKernel_);
  }
  if (!s.ok) return s;
  {
    std::vector<CD3DX12_ROOT_PARAMETER1> params(3);
    params[0].InitAsConstants(4, 0);
    params[1].InitAsUnorderedAccessView(4);
    params[2].InitAsUnorderedAccessView(5);
    s = CompileKernel(L"Scan", params, scanKernel_);
  }
  if (!s.ok) return s;
  {
    std::vector<CD3DX12_ROOT_PARAMETER1> params(7);
    params[0].InitAsConstants(4, 0);
    params[1].InitAsUnorderedAccessView(0);
    params[2].InitAsUnorderedAccessView(1);
    params[3].InitAsUnorderedAccessView(2);
    params[4].InitAsUnorderedAccessView(3);
    params[5].InitAsUnorderedAccessView(5);
    params[6].InitAsUnorderedAccessView(6);
    s = CompileKernel(L"DigitBinningPass", params, digitBinningKernel_);
  }
  return s;
}

Status OneSweep::Dispatch(const OneSweepDispatch& dispatch, OneSweepResult& outResult) const {
  outResult = {};
  if (!IsInitialized()) {
    return Status::Error("OneSweep backend is not initialized");
  }
  if (dispatch.commandList == nullptr || dispatch.keyPrimaryResource == nullptr || dispatch.keyPrimaryUav == 0 ||
      dispatch.keyTempResource == nullptr || dispatch.keyTempUav == 0 || dispatch.valuePrimaryResource == nullptr ||
      dispatch.valuePrimaryUav == 0 || dispatch.valueTempResource == nullptr || dispatch.valueTempUav == 0 ||
      dispatch.passHistogramResource == nullptr || dispatch.passHistogramUav == 0 ||
      dispatch.globalHistogramResource == nullptr || dispatch.globalHistogramUav == 0 || dispatch.indexResource == nullptr ||
      dispatch.indexUav == 0 || dispatch.elementCount == 0) {
    return Status::Error("invalid OneSweep dispatch");
  }

  const uint32_t partitionSize = std::max<uint32_t>(tuning_.partitionSize, 1u);
  const uint32_t partitions = DivRoundUp(dispatch.elementCount, partitionSize);
  const uint32_t globalHistPartitions = DivRoundUp(dispatch.elementCount, kGlobalHistPartitionSize);

  auto setKernel = [&](const Kernel& kernel) {
    dispatch.commandList->SetComputeRootSignature(kernel.rootSignature.Get());
    dispatch.commandList->SetPipelineState(kernel.pipelineState.Get());
  };

  setKernel(*initSweepKernel_);
  {
    const uint32_t constants[4] = {0u, 0u, partitions, 0u};
    dispatch.commandList->SetComputeRoot32BitConstants(0, 4, constants, 0);
    dispatch.commandList->SetComputeRootUnorderedAccessView(1, dispatch.globalHistogramUav);
    dispatch.commandList->SetComputeRootUnorderedAccessView(2, dispatch.passHistogramUav);
    dispatch.commandList->SetComputeRootUnorderedAccessView(3, dispatch.indexUav);
    dispatch.commandList->Dispatch(256u, 1u, 1u);
  }
  UavBarrier(dispatch.commandList, dispatch.globalHistogramResource);
  UavBarrier(dispatch.commandList, dispatch.passHistogramResource);
  UavBarrier(dispatch.commandList, dispatch.indexResource);

  setKernel(*globalHistogramKernel_);
  {
    const uint32_t fullBlocks = globalHistPartitions / kMaxDispatchDimension;
    if (fullBlocks > 0) {
      const uint32_t constants[4] = {dispatch.elementCount, 0u, globalHistPartitions, 0u};
      dispatch.commandList->SetComputeRoot32BitConstants(0, 4, constants, 0);
      dispatch.commandList->SetComputeRootUnorderedAccessView(1, dispatch.keyPrimaryUav);
      dispatch.commandList->SetComputeRootUnorderedAccessView(2, dispatch.globalHistogramUav);
      dispatch.commandList->Dispatch(kMaxDispatchDimension, fullBlocks, 1u);
    }
    const uint32_t partialBlocks = globalHistPartitions - fullBlocks * kMaxDispatchDimension;
    if (partialBlocks > 0) {
      const uint32_t constants[4] = {dispatch.elementCount, 0u, globalHistPartitions, (fullBlocks << 1u) | 1u};
      dispatch.commandList->SetComputeRoot32BitConstants(0, 4, constants, 0);
      dispatch.commandList->SetComputeRootUnorderedAccessView(1, dispatch.keyPrimaryUav);
      dispatch.commandList->SetComputeRootUnorderedAccessView(2, dispatch.globalHistogramUav);
      dispatch.commandList->Dispatch(partialBlocks, 1u, 1u);
    }
  }
  UavBarrier(dispatch.commandList, dispatch.globalHistogramResource);

  setKernel(*scanKernel_);
  {
    const uint32_t constants[4] = {0u, 0u, partitions, 0u};
    dispatch.commandList->SetComputeRoot32BitConstants(0, 4, constants, 0);
    dispatch.commandList->SetComputeRootUnorderedAccessView(1, dispatch.globalHistogramUav);
    dispatch.commandList->SetComputeRootUnorderedAccessView(2, dispatch.passHistogramUav);
    dispatch.commandList->Dispatch(kRadixPasses, 1u, 1u);
  }
  UavBarrier(dispatch.commandList, dispatch.passHistogramResource);

  ID3D12Resource* srcKeyResource = dispatch.keyPrimaryResource;
  ID3D12Resource* dstKeyResource = dispatch.keyTempResource;
  ID3D12Resource* srcValueResource = dispatch.valuePrimaryResource;
  ID3D12Resource* dstValueResource = dispatch.valueTempResource;
  D3D12_GPU_VIRTUAL_ADDRESS srcKeyUav = dispatch.keyPrimaryUav;
  D3D12_GPU_VIRTUAL_ADDRESS dstKeyUav = dispatch.keyTempUav;
  D3D12_GPU_VIRTUAL_ADDRESS srcValueUav = dispatch.valuePrimaryUav;
  D3D12_GPU_VIRTUAL_ADDRESS dstValueUav = dispatch.valueTempUav;
  bool sortedInPrimary = true;

  setKernel(*digitBinningKernel_);
  for (uint32_t radixShift = 0u; radixShift < 32u; radixShift += 8u) {
    const uint32_t fullBlocks = partitions / kMaxDispatchDimension;
    if (fullBlocks > 0) {
      const uint32_t constants[4] = {dispatch.elementCount, radixShift, partitions, 0u};
      dispatch.commandList->SetComputeRoot32BitConstants(0, 4, constants, 0);
      dispatch.commandList->SetComputeRootUnorderedAccessView(1, srcKeyUav);
      dispatch.commandList->SetComputeRootUnorderedAccessView(2, dstKeyUav);
      dispatch.commandList->SetComputeRootUnorderedAccessView(3, srcValueUav);
      dispatch.commandList->SetComputeRootUnorderedAccessView(4, dstValueUav);
      dispatch.commandList->SetComputeRootUnorderedAccessView(5, dispatch.passHistogramUav);
      dispatch.commandList->SetComputeRootUnorderedAccessView(6, dispatch.indexUav);
      dispatch.commandList->Dispatch(kMaxDispatchDimension, fullBlocks, 1u);
      UavBarrier(dispatch.commandList, dispatch.passHistogramResource);
    }

    const uint32_t partialBlocks = partitions - fullBlocks * kMaxDispatchDimension;
    if (partialBlocks > 0) {
      const uint32_t constants[4] = {dispatch.elementCount, radixShift, partitions, 0u};
      dispatch.commandList->SetComputeRoot32BitConstants(0, 4, constants, 0);
      dispatch.commandList->SetComputeRootUnorderedAccessView(1, srcKeyUav);
      dispatch.commandList->SetComputeRootUnorderedAccessView(2, dstKeyUav);
      dispatch.commandList->SetComputeRootUnorderedAccessView(3, srcValueUav);
      dispatch.commandList->SetComputeRootUnorderedAccessView(4, dstValueUav);
      dispatch.commandList->SetComputeRootUnorderedAccessView(5, dispatch.passHistogramUav);
      dispatch.commandList->SetComputeRootUnorderedAccessView(6, dispatch.indexUav);
      dispatch.commandList->Dispatch(partialBlocks, 1u, 1u);
    }

    UavBarrier(dispatch.commandList, srcKeyResource);
    UavBarrier(dispatch.commandList, srcValueResource);
    UavBarrier(dispatch.commandList, dstKeyResource);
    UavBarrier(dispatch.commandList, dstValueResource);

    std::swap(srcKeyResource, dstKeyResource);
    std::swap(srcValueResource, dstValueResource);
    std::swap(srcKeyUav, dstKeyUav);
    std::swap(srcValueUav, dstValueUav);
    sortedInPrimary = !sortedInPrimary;
  }

  outResult.sortedInPrimary = sortedInPrimary;
  outResult.passCount = kRadixPasses;
  return Status::Ok();
}

Status OneSweep::DispatchIndirect(const OneSweepIndirectDispatch& dispatch, OneSweepResult& outResult) const {
  outResult = {};
  if (!IsInitialized()) {
    return Status::Error("OneSweep backend is not initialized");
  }
  if (dispatch.commandList == nullptr || dispatch.keyPrimaryResource == nullptr || dispatch.keyPrimaryUav == 0 ||
      dispatch.keyTempResource == nullptr || dispatch.keyTempUav == 0 || dispatch.valuePrimaryResource == nullptr ||
      dispatch.valuePrimaryUav == 0 || dispatch.valueTempResource == nullptr || dispatch.valueTempUav == 0 ||
      dispatch.passHistogramResource == nullptr || dispatch.passHistogramUav == 0 ||
      dispatch.globalHistogramResource == nullptr || dispatch.globalHistogramUav == 0 || dispatch.indexResource == nullptr ||
      dispatch.indexUav == 0 || dispatch.argumentBufferResource == nullptr) {
    return Status::Error("invalid OneSweep indirect dispatch");
  }

  auto setKernel = [&](const Kernel& kernel) {
    dispatch.commandList->SetComputeRootSignature(kernel.rootSignature.Get());
    dispatch.commandList->SetPipelineState(kernel.pipelineState.Get());
  };

  setKernel(*initSweepKernel_);
  dispatch.commandList->SetComputeRootUnorderedAccessView(1, dispatch.globalHistogramUav);
  dispatch.commandList->SetComputeRootUnorderedAccessView(2, dispatch.passHistogramUav);
  dispatch.commandList->SetComputeRootUnorderedAccessView(3, dispatch.indexUav);
  dispatch.commandList->ExecuteIndirect(initSweepKernel_->indirectCommandSignature.Get(), 1, dispatch.argumentBufferResource,
                                        dispatch.initArgsOffset, nullptr, 0);
  UavBarrier(dispatch.commandList, dispatch.globalHistogramResource);
  UavBarrier(dispatch.commandList, dispatch.passHistogramResource);
  UavBarrier(dispatch.commandList, dispatch.indexResource);

  setKernel(*globalHistogramKernel_);
  dispatch.commandList->SetComputeRootUnorderedAccessView(1, dispatch.keyPrimaryUav);
  dispatch.commandList->SetComputeRootUnorderedAccessView(2, dispatch.globalHistogramUav);
  dispatch.commandList->ExecuteIndirect(globalHistogramKernel_->indirectCommandSignature.Get(), 1, dispatch.argumentBufferResource,
                                        dispatch.histogramArgsOffset, nullptr, 0);
  UavBarrier(dispatch.commandList, dispatch.globalHistogramResource);

  setKernel(*scanKernel_);
  dispatch.commandList->SetComputeRootUnorderedAccessView(1, dispatch.globalHistogramUav);
  dispatch.commandList->SetComputeRootUnorderedAccessView(2, dispatch.passHistogramUav);
  dispatch.commandList->ExecuteIndirect(scanKernel_->indirectCommandSignature.Get(), 1, dispatch.argumentBufferResource,
                                        dispatch.scanArgsOffset, nullptr, 0);
  UavBarrier(dispatch.commandList, dispatch.passHistogramResource);

  ID3D12Resource* srcKeyResource = dispatch.keyPrimaryResource;
  ID3D12Resource* dstKeyResource = dispatch.keyTempResource;
  ID3D12Resource* srcValueResource = dispatch.valuePrimaryResource;
  ID3D12Resource* dstValueResource = dispatch.valueTempResource;
  D3D12_GPU_VIRTUAL_ADDRESS srcKeyUav = dispatch.keyPrimaryUav;
  D3D12_GPU_VIRTUAL_ADDRESS dstKeyUav = dispatch.keyTempUav;
  D3D12_GPU_VIRTUAL_ADDRESS srcValueUav = dispatch.valuePrimaryUav;
  D3D12_GPU_VIRTUAL_ADDRESS dstValueUav = dispatch.valueTempUav;
  bool sortedInPrimary = true;

  setKernel(*digitBinningKernel_);
  const uint32_t digitPassCount = std::max<uint32_t>(dispatch.digitPassCount, 1u);
  for (uint32_t passIndex = 0; passIndex < digitPassCount; ++passIndex) {
    dispatch.commandList->SetComputeRootUnorderedAccessView(1, srcKeyUav);
    dispatch.commandList->SetComputeRootUnorderedAccessView(2, dstKeyUav);
    dispatch.commandList->SetComputeRootUnorderedAccessView(3, srcValueUav);
    dispatch.commandList->SetComputeRootUnorderedAccessView(4, dstValueUav);
    dispatch.commandList->SetComputeRootUnorderedAccessView(5, dispatch.passHistogramUav);
    dispatch.commandList->SetComputeRootUnorderedAccessView(6, dispatch.indexUav);
    dispatch.commandList->ExecuteIndirect(digitBinningKernel_->indirectCommandSignature.Get(), 1, dispatch.argumentBufferResource,
                                          dispatch.digitArgsOffset + static_cast<uint64_t>(passIndex) * sizeof(uint32_t) * 7u,
                                          nullptr, 0);
    UavBarrier(dispatch.commandList, dispatch.passHistogramResource);
    UavBarrier(dispatch.commandList, srcKeyResource);
    UavBarrier(dispatch.commandList, srcValueResource);
    UavBarrier(dispatch.commandList, dstKeyResource);
    UavBarrier(dispatch.commandList, dstValueResource);
    std::swap(srcKeyResource, dstKeyResource);
    std::swap(srcValueResource, dstValueResource);
    std::swap(srcKeyUav, dstKeyUav);
    std::swap(srcValueUav, dstValueUav);
    sortedInPrimary = !sortedInPrimary;
  }

  outResult.sortedInPrimary = sortedInPrimary;
  outResult.passCount = digitPassCount;
  return Status::Ok();
}

}  // namespace directxsplat
