include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

get_filename_component(DIRECTXSPLAT_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(DIRECTXSPLAT_SHADER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/shaders")
set(DIRECTXSPLAT_CMAKE_DIR "${DIRECTXSPLAT_ROOT_DIR}/cmake")
set(DIRECTXSPLAT_3RDPARTY_DIR "${DIRECTXSPLAT_ROOT_DIR}/3rdparty/GPUSorting/GPUSortingD3D12")
set(DIRECTXSPLAT_3RDPARTY_SHADER_DIR "${DIRECTXSPLAT_3RDPARTY_DIR}/Shaders")
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(DIRECTXSPLAT_ARCH_DIR x64)
else()
  set(DIRECTXSPLAT_ARCH_DIR x86)
endif()

set(DIRECTXSPLAT_VIEWER_RUNTIME_NAMES
  msvcp140.dll
  vcruntime140.dll
  vcruntime140_1.dll
)
set(DIRECTXSPLAT_VIEWER_RUNTIME_DLLS "")
set(DIRECTXSPLAT_VIEWER_RUNTIME_FOUND_NAMES "")
set(DIRECTXSPLAT_VIEWER_RUNTIME_COMPLETE FALSE)
if(DIRECTXSPLAT_BUILD_VIEWER)
  set(_directxsplat_runtime_skip_was_defined FALSE)
  if(DEFINED CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_SKIP)
    set(_directxsplat_runtime_skip_was_defined TRUE)
    set(_directxsplat_runtime_skip "${CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_SKIP}")
  endif()
  set(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_SKIP TRUE)
  include(InstallRequiredSystemLibraries)
  if(_directxsplat_runtime_skip_was_defined)
    set(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_SKIP "${_directxsplat_runtime_skip}")
  else()
    unset(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_SKIP)
  endif()
  unset(_directxsplat_runtime_skip)
  unset(_directxsplat_runtime_skip_was_defined)

  foreach(runtime_dll IN LISTS CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS)
    get_filename_component(runtime_name "${runtime_dll}" NAME)
    string(TOLOWER "${runtime_name}" runtime_name)
    if(runtime_name IN_LIST DIRECTXSPLAT_VIEWER_RUNTIME_NAMES)
      list(APPEND DIRECTXSPLAT_VIEWER_RUNTIME_DLLS "${runtime_dll}")
      list(APPEND DIRECTXSPLAT_VIEWER_RUNTIME_FOUND_NAMES "${runtime_name}")
    endif()
  endforeach()
  set(DIRECTXSPLAT_VIEWER_RUNTIME_COMPLETE TRUE)
  foreach(runtime_name IN LISTS DIRECTXSPLAT_VIEWER_RUNTIME_NAMES)
    if(NOT runtime_name IN_LIST DIRECTXSPLAT_VIEWER_RUNTIME_FOUND_NAMES)
      set(DIRECTXSPLAT_VIEWER_RUNTIME_COMPLETE FALSE)
    endif()
  endforeach()
endif()

set(DIRECTXSPLAT_DXCOMPILER_DLL "")
set(DIRECTXSPLAT_DXIL_DLL "")
file(GLOB DIRECTXSPLAT_DXCOMPILER_CANDIDATES
  "C:/Program Files (x86)/Windows Kits/10/Redist/D3D/${DIRECTXSPLAT_ARCH_DIR}/dxcompiler.dll"
  "C:/Program Files (x86)/Windows Kits/10/bin/*/${DIRECTXSPLAT_ARCH_DIR}/dxcompiler.dll"
)
list(SORT DIRECTXSPLAT_DXCOMPILER_CANDIDATES COMPARE NATURAL ORDER DESCENDING)
foreach(candidate IN LISTS DIRECTXSPLAT_DXCOMPILER_CANDIDATES)
  if(EXISTS "${candidate}")
    set(DIRECTXSPLAT_DXCOMPILER_DLL "${candidate}")
    break()
  endif()
endforeach()
file(GLOB DIRECTXSPLAT_DXIL_CANDIDATES
  "C:/Program Files (x86)/Windows Kits/10/Redist/D3D/${DIRECTXSPLAT_ARCH_DIR}/dxil.dll"
  "C:/Program Files (x86)/Windows Kits/10/bin/*/${DIRECTXSPLAT_ARCH_DIR}/dxil.dll"
)
list(SORT DIRECTXSPLAT_DXIL_CANDIDATES COMPARE NATURAL ORDER DESCENDING)
foreach(candidate IN LISTS DIRECTXSPLAT_DXIL_CANDIDATES)
  if(EXISTS "${candidate}")
    set(DIRECTXSPLAT_DXIL_DLL "${candidate}")
    break()
  endif()
endforeach()
set_property(GLOBAL PROPERTY DIRECTXSPLAT_DXCOMPILER_DLL "${DIRECTXSPLAT_DXCOMPILER_DLL}")
set_property(GLOBAL PROPERTY DIRECTXSPLAT_DXIL_DLL "${DIRECTXSPLAT_DXIL_DLL}")

file(GLOB_RECURSE DIRECTXSPLAT_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
list(REMOVE_ITEM DIRECTXSPLAT_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/app/main.cpp)

get_property(DIRECTXSPLAT_IMGUI_SOURCE_DIR GLOBAL PROPERTY DIRECTXSPLAT_IMGUI_SOURCE_DIR)
if(NOT DIRECTXSPLAT_IMGUI_SOURCE_DIR)
  message(FATAL_ERROR "DirectXSplat requires ImGui sources to build the owned viewer runtime.")
endif()
set(DIRECTXSPLAT_IMGUI_SOURCES
  ${DIRECTXSPLAT_IMGUI_SOURCE_DIR}/imgui.cpp
  ${DIRECTXSPLAT_IMGUI_SOURCE_DIR}/imgui_draw.cpp
  ${DIRECTXSPLAT_IMGUI_SOURCE_DIR}/imgui_tables.cpp
  ${DIRECTXSPLAT_IMGUI_SOURCE_DIR}/imgui_widgets.cpp
  ${DIRECTXSPLAT_IMGUI_SOURCE_DIR}/imgui_demo.cpp
  ${DIRECTXSPLAT_IMGUI_SOURCE_DIR}/backends/imgui_impl_dx12.cpp
  ${DIRECTXSPLAT_IMGUI_SOURCE_DIR}/backends/imgui_impl_win32.cpp
)
set(DIRECTXSPLAT_THIRDPARTY_SOURCES
  ${DIRECTXSPLAT_ROOT_DIR}/3rdparty/miniz.c
  ${DIRECTXSPLAT_ROOT_DIR}/3rdparty/miniz_tdef.c
  ${DIRECTXSPLAT_ROOT_DIR}/3rdparty/miniz_tinfl.c
  ${DIRECTXSPLAT_ROOT_DIR}/3rdparty/miniz_zip.c
)
list(APPEND DIRECTXSPLAT_SOURCES ${DIRECTXSPLAT_THIRDPARTY_SOURCES} ${DIRECTXSPLAT_IMGUI_SOURCES})

if(MSVC)
  set_source_files_properties(${DIRECTXSPLAT_THIRDPARTY_SOURCES} ${DIRECTXSPLAT_IMGUI_SOURCES} PROPERTIES COMPILE_OPTIONS /W0)
endif()

set(DIRECTXSPLAT_EMBEDDED_SHADER_HEADER ${CMAKE_CURRENT_BINARY_DIR}/generated/EmbeddedShaders.h)
add_custom_command(
  OUTPUT ${DIRECTXSPLAT_EMBEDDED_SHADER_HEADER}
  COMMAND ${CMAKE_COMMAND}
    -DOUTPUT=${DIRECTXSPLAT_EMBEDDED_SHADER_HEADER}
    -DCOMPUTE_INPUT=${DIRECTXSPLAT_SHADER_DIR}/gaussian_compute.hlsl
    -DRASTER_INPUT=${DIRECTXSPLAT_SHADER_DIR}/gaussian_raster.hlsl
    -DONESWEEP_INPUT=${DIRECTXSPLAT_3RDPARTY_SHADER_DIR}/OneSweep.hlsl
    -DSWEEP_COMMON_INPUT=${DIRECTXSPLAT_3RDPARTY_SHADER_DIR}/SweepCommon.hlsl
    -DSORT_COMMON_INPUT=${DIRECTXSPLAT_3RDPARTY_SHADER_DIR}/SortCommon.hlsl
    -P ${DIRECTXSPLAT_CMAKE_DIR}/GenerateEmbeddedShaders.cmake
  DEPENDS
    ${DIRECTXSPLAT_CMAKE_DIR}/GenerateEmbeddedShaders.cmake
    ${DIRECTXSPLAT_SHADER_DIR}/gaussian_compute.hlsl
    ${DIRECTXSPLAT_SHADER_DIR}/gaussian_raster.hlsl
    ${DIRECTXSPLAT_3RDPARTY_SHADER_DIR}/OneSweep.hlsl
    ${DIRECTXSPLAT_3RDPARTY_SHADER_DIR}/SweepCommon.hlsl
    ${DIRECTXSPLAT_3RDPARTY_SHADER_DIR}/SortCommon.hlsl
)

list(APPEND DIRECTXSPLAT_SOURCES ${DIRECTXSPLAT_EMBEDDED_SHADER_HEADER})

add_library(directxsplat STATIC ${DIRECTXSPLAT_SOURCES})
add_library(directxsplat::directxsplat ALIAS directxsplat)
add_library(DirectXSplat::DirectXSplat ALIAS directxsplat)

set_target_properties(directxsplat PROPERTIES
  OUTPUT_NAME DirectXSplat
  EXPORT_NAME DirectXSplat
)

target_compile_features(directxsplat PUBLIC cxx_std_20)

if(COMMAND directxsplat_apply_project_warnings)
  directxsplat_apply_project_warnings(directxsplat)
endif()

target_include_directories(directxsplat
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/app
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/core
    ${CMAKE_CURRENT_BINARY_DIR}/generated
)

target_include_directories(directxsplat SYSTEM
  PRIVATE
    ${DIRECTXSPLAT_ROOT_DIR}/3rdparty
    ${DIRECTXSPLAT_3RDPARTY_DIR}
    ${DIRECTXSPLAT_IMGUI_SOURCE_DIR}
    ${DIRECTXSPLAT_IMGUI_SOURCE_DIR}/backends
)

if(NOT TARGET nlohmann_json::nlohmann_json)
  find_package(nlohmann_json CONFIG REQUIRED)
endif()

get_target_property(DIRECTXSPLAT_JSON_INCLUDE_DIRS
  nlohmann_json::nlohmann_json
  INTERFACE_INCLUDE_DIRECTORIES)
if(DIRECTXSPLAT_JSON_INCLUDE_DIRS)
  target_include_directories(directxsplat SYSTEM PRIVATE ${DIRECTXSPLAT_JSON_INCLUDE_DIRS})
endif()

target_compile_definitions(directxsplat
  PRIVATE
    UNICODE
    _UNICODE
    WIN32_LEAN_AND_MEAN
    IMGUI_IMPL_WIN32_DISABLE_GAMEPAD
  PUBLIC
    NOMINMAX
)

target_link_libraries(directxsplat
  PUBLIC
    d3d12
    dxgi
    dxguid
    d3dcompiler
    dxcompiler
    windowscodecs
    ole32
    dwmapi
    shell32
    comdlg32
)

if(DIRECTXSPLAT_BUILD_VIEWER)
  if(NOT WIN32)
    message(STATUS "directxsplat_viewer skipped: requires WIN32")
  else()
    add_executable(directxsplat_viewer WIN32 src/app/main.cpp)

    set_target_properties(directxsplat_viewer PROPERTIES OUTPUT_NAME DirectXSplatViewer)

    target_include_directories(directxsplat_viewer
      PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/app
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    if(COMMAND directxsplat_apply_project_warnings)
      directxsplat_apply_project_warnings(directxsplat_viewer)
    endif()

    target_link_libraries(directxsplat_viewer
      PRIVATE
        DirectXSplat::DirectXSplat
    )

    if(COMMAND directxsplat_stage_dxc_runtime)
      directxsplat_stage_dxc_runtime(directxsplat_viewer)
    endif()

    if(DIRECTXSPLAT_VIEWER_RUNTIME_COMPLETE)
      install(FILES
        ${DIRECTXSPLAT_VIEWER_RUNTIME_DLLS}
        DESTINATION ${CMAKE_INSTALL_BINDIR}
      )
    endif()

    install(TARGETS directxsplat_viewer
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    install(FILES
      ${DIRECTXSPLAT_ROOT_DIR}/LICENSE
      DESTINATION .
    )
    install(FILES
      ${DIRECTXSPLAT_ROOT_DIR}/README.md
      DESTINATION .
      RENAME README.txt
    )

    if(DIRECTXSPLAT_DXCOMPILER_DLL AND DIRECTXSPLAT_DXIL_DLL AND DIRECTXSPLAT_VIEWER_RUNTIME_COMPLETE)
      set(DIRECTXSPLAT_VIEWER_PACKAGE_NAME "DirectXSplatViewer-${PROJECT_VERSION}-win64")
      set(DIRECTXSPLAT_VIEWER_PACKAGE_DIR "${CMAKE_BINARY_DIR}/${DIRECTXSPLAT_VIEWER_PACKAGE_NAME}")
      set(DIRECTXSPLAT_VIEWER_PACKAGE_ZIP "${CMAKE_BINARY_DIR}/${DIRECTXSPLAT_VIEWER_PACKAGE_NAME}.zip")

      add_custom_command(
        OUTPUT "${DIRECTXSPLAT_VIEWER_PACKAGE_ZIP}"
        COMMAND ${CMAKE_COMMAND} -E rm -rf "${DIRECTXSPLAT_VIEWER_PACKAGE_DIR}"
        COMMAND ${CMAKE_COMMAND} -E make_directory "${DIRECTXSPLAT_VIEWER_PACKAGE_DIR}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:directxsplat_viewer>" "${DIRECTXSPLAT_VIEWER_PACKAGE_DIR}/DirectXSplatViewer.exe"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${DIRECTXSPLAT_DXCOMPILER_DLL}" "${DIRECTXSPLAT_VIEWER_PACKAGE_DIR}/dxcompiler.dll"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${DIRECTXSPLAT_DXIL_DLL}" "${DIRECTXSPLAT_VIEWER_PACKAGE_DIR}/dxil.dll"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DIRECTXSPLAT_VIEWER_RUNTIME_DLLS} "${DIRECTXSPLAT_VIEWER_PACKAGE_DIR}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${DIRECTXSPLAT_ROOT_DIR}/LICENSE" "${DIRECTXSPLAT_VIEWER_PACKAGE_DIR}/LICENSE"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different "${DIRECTXSPLAT_ROOT_DIR}/README.md" "${DIRECTXSPLAT_VIEWER_PACKAGE_DIR}/README.txt"
        COMMAND ${CMAKE_COMMAND} -E rm -f "${DIRECTXSPLAT_VIEWER_PACKAGE_ZIP}"
        COMMAND ${CMAKE_COMMAND} -E chdir "${DIRECTXSPLAT_VIEWER_PACKAGE_DIR}" ${CMAKE_COMMAND} -E tar cf "${DIRECTXSPLAT_VIEWER_PACKAGE_ZIP}" --format=zip -- DirectXSplatViewer.exe dxcompiler.dll dxil.dll ${DIRECTXSPLAT_VIEWER_RUNTIME_NAMES} LICENSE README.txt
        DEPENDS
          directxsplat_viewer
          "${DIRECTXSPLAT_DXCOMPILER_DLL}"
          "${DIRECTXSPLAT_DXIL_DLL}"
          ${DIRECTXSPLAT_VIEWER_RUNTIME_DLLS}
          "${DIRECTXSPLAT_ROOT_DIR}/LICENSE"
          "${DIRECTXSPLAT_ROOT_DIR}/README.md"
        VERBATIM
      )
      add_custom_target(directxsplat_viewer_package DEPENDS "${DIRECTXSPLAT_VIEWER_PACKAGE_ZIP}")
    else()
      message(WARNING "directxsplat_viewer_package skipped: a required runtime DLL was not found")
    endif()
  endif()
endif()

set(directxsplat_install_cmakedir ${CMAKE_INSTALL_LIBDIR}/cmake/DirectXSplat)
file(GLOB DIRECTXSPLAT_PUBLIC_HEADERS CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/directxsplat/*.h
)

install(TARGETS directxsplat
  EXPORT DirectXSplatTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(FILES
  ${DIRECTXSPLAT_PUBLIC_HEADERS}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/directxsplat
)

install(EXPORT DirectXSplatTargets
  FILE DirectXSplatTargets.cmake
  NAMESPACE DirectXSplat::
  DESTINATION ${directxsplat_install_cmakedir}
)

configure_package_config_file(
  ${DIRECTXSPLAT_CMAKE_DIR}/DirectXSplatConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/DirectXSplatConfig.cmake
  INSTALL_DESTINATION ${directxsplat_install_cmakedir}
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/DirectXSplatConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/DirectXSplatConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/DirectXSplatConfigVersion.cmake
  DESTINATION ${directxsplat_install_cmakedir}
)

if(DIRECTXSPLAT_DXCOMPILER_DLL AND DIRECTXSPLAT_DXIL_DLL)
  install(FILES
    ${DIRECTXSPLAT_DXCOMPILER_DLL}
    ${DIRECTXSPLAT_DXIL_DLL}
    DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
endif()
