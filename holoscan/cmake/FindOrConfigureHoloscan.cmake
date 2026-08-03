# SPDX-License-Identifier: Apache-2.0
#
# Resolve the in-development Holoscan SDK 5.x build tree and define the imported
# targets holoscan::core / holoscan::logger / holoscan::schema by reusing the
# SDK's own exported holoscan-targets.cmake.
#
# Why not just find_package(holoscan)?
#   The SDK build tree's holoscan-config.cmake include()s a
#   holoscan-full-config.cmake that is only generated for the SDK's in-tree
#   build, so a downstream find_package() against build-host fails outright.
#   The relocatable install tree does ship a working config, but it is a stale
#   snapshot with the older runtime API. We track whatever the user last built
#   under public/build-host by satisfying the transitive dependency targets
#   ourselves and including the exported targets file directly.
#
# Cache variables (override on the cmake command line):
#   HOLOSCAN_INSTALL_ROOT  path to an installed SDK prefix (also honoured from
#                          the environment; autodetected under $HOME when no
#                          usable build tree exists)
#   HOLOSCAN_SDK_ROOT   path to holoscan-main-5x/public
#                       (default: sibling ../holoscan-main-5x/public)
#   HOLOSCAN_BUILD_DIR  build dir name under public/, or an absolute path
#                       (default: build-host)
#
# On success defines: holoscan::core, holoscan::logger, holoscan::schema and the
# cache variable HOLOSCAN_LIB_DIR (runtime .so location, for RPATH).

if(TARGET holoscan::core)
  return()
endif()

set(HOLOSCAN_INSTALL_ROOT "" CACHE PATH "Path to an installed Holoscan SDK prefix")
set(HOLOSCAN_SDK_ROOT "" CACHE PATH "Path to the holoscan-main-5x/public tree")
set(HOLOSCAN_BUILD_DIR "build-host" CACHE STRING
  "Holoscan build dir under public/ (a name) or an absolute path")

if(NOT HOLOSCAN_INSTALL_ROOT AND Holoscan_ROOT)
  set(HOLOSCAN_INSTALL_ROOT "${Holoscan_ROOT}")
endif()
if(NOT HOLOSCAN_INSTALL_ROOT AND DEFINED ENV{HOLOSCAN_INSTALL_ROOT})
  set(HOLOSCAN_INSTALL_ROOT "$ENV{HOLOSCAN_INSTALL_ROOT}")
endif()

# --- 1. SDK source root ------------------------------------------------------
if(NOT HOLOSCAN_SDK_ROOT)
  # This module lives in scorbot/holoscan/cmake, so the SDK checkout is three
  # levels up rather than the two the examples repo needs.
  foreach(_holo_rel "../../../holoscan-main-5x/public" "../../holoscan-main-5x/public")
    get_filename_component(_holo_default_root "${CMAKE_CURRENT_LIST_DIR}/${_holo_rel}" ABSOLUTE)
    if(EXISTS "${_holo_default_root}/include/holoscan/holoscan.hpp")
      set(HOLOSCAN_SDK_ROOT "${_holo_default_root}")
      break()
    endif()
  endforeach()
endif()

# --- 2. SDK build dir --------------------------------------------------------
if(IS_ABSOLUTE "${HOLOSCAN_BUILD_DIR}")
  set(_holo_build "${HOLOSCAN_BUILD_DIR}")
else()
  set(_holo_build "${HOLOSCAN_SDK_ROOT}/${HOLOSCAN_BUILD_DIR}")
endif()

# --- 3. Autodetect an install prefix when the build tree is unusable ---------
# On native aarch64 targets (Thor/Jetson) the SDK is consumed from `cmake
# --install` output, because its build tree writes no export at the build root.
# /opt/nvidia/holoscan is deliberately never probed: that is Holoscan 4.4, a
# different API.
if(NOT HOLOSCAN_INSTALL_ROOT AND NOT EXISTS "${_holo_build}/holoscan-targets.cmake")
  cmake_host_system_information(RESULT _holo_host_arch QUERY OS_PLATFORM)
  foreach(_holo_candidate
      "$ENV{HOME}/holoscan-install-${_holo_host_arch}"
      "$ENV{HOME}/holoscan-install")
    if(EXISTS "${_holo_candidate}/lib/cmake/holoscan/holoscan-config.cmake")
      set(HOLOSCAN_INSTALL_ROOT "${_holo_candidate}")
      message(STATUS "Holoscan SDK install autodetected: ${_holo_candidate}")
      break()
    endif()
  endforeach()
endif()

# --- 4. Consume the install prefix, if we have one ---------------------------
if(HOLOSCAN_INSTALL_ROOT)
  find_package(holoscan CONFIG REQUIRED
    PATHS "${HOLOSCAN_INSTALL_ROOT}"
    NO_DEFAULT_PATH)
  set(HOLOSCAN_LIB_DIR "${HOLOSCAN_INSTALL_ROOT}/lib" CACHE INTERNAL
    "Holoscan runtime library dir")
  message(STATUS "Holoscan SDK install resolved: ${HOLOSCAN_INSTALL_ROOT}")
  return()
endif()

# --- 5. Otherwise the in-development build tree is required ------------------
# Missing the SDK is not fatal here: the comms layer, tools and web UI do not
# depend on Holoscan, so only the operators and example app get skipped.
if(NOT HOLOSCAN_SDK_ROOT OR NOT EXISTS "${HOLOSCAN_SDK_ROOT}/include/holoscan/holoscan.hpp")
  message(WARNING
    "Holoscan SDK source not found. Pass -DHOLOSCAN_SDK_ROOT=/path/to/holoscan-main-5x/public "
    "or -DHOLOSCAN_INSTALL_ROOT=/path/to/installed/prefix")
  return()
endif()
if(NOT EXISTS "${_holo_build}/holoscan-targets.cmake")
  message(WARNING
    "No holoscan-targets.cmake under ${_holo_build}. "
    "Build the SDK first, set -DHOLOSCAN_BUILD_DIR=<dir>, or point "
    "-DHOLOSCAN_INSTALL_ROOT=<prefix> at an installed SDK.")
  return()
endif()
set(HOLOSCAN_LIB_DIR "${_holo_build}/lib" CACHE INTERNAL "Holoscan runtime library dir")

# --- 3. CUDA toolkit + CCCL --------------------------------------------------
find_package(CUDAToolkit REQUIRED)
if(NOT TARGET CCCL::CCCL)
  find_package(CCCL CONFIG QUIET
    PATHS
      "${CUDAToolkit_LIBRARY_DIR}/cmake"
      "${CUDAToolkit_LIBRARY_ROOT}/lib64/cmake"
      "/usr/local/cuda/lib64/cmake"
      "${_holo_build}/_deps/cccl-build")
endif()
if(NOT TARGET CCCL::CCCL)
  add_library(CCCL::CCCL INTERFACE IMPORTED)
  set_target_properties(CCCL::CCCL PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CUDAToolkit_INCLUDE_DIRS}")
endif()

# --- 4. Header-only deps (fmt / dlpack / magic_enum / tl-expected) -----------
# These live in the holomq CPM cache under a per-content-hash dir; glob so a
# rebuild that changes the hash still resolves. SDK _deps are a fallback.
function(_holoscan_header_dep target)
  if(TARGET ${target})
    return()
  endif()
  set(_inc "")
  foreach(_pattern IN LISTS ARGN)
    file(GLOB _hits "${_pattern}")
    foreach(_h IN LISTS _hits)
      if(IS_DIRECTORY "${_h}")
        set(_inc "${_h}")
        break()
      endif()
    endforeach()
    if(_inc)
      break()
    endif()
  endforeach()
  if(NOT _inc)
    message(FATAL_ERROR "Could not resolve an include dir for ${target}. Searched: ${ARGN}")
  endif()
  add_library(${target} INTERFACE IMPORTED)
  set_target_properties(${target} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_inc}")
endfunction()

set(_holo_cpm "$ENV{HOME}/.cache/holomq_host_cpm")
_holoscan_header_dep(fmt::fmt-header-only
  "${_holo_cpm}/fmt/*/include" "${_holo_build}/_deps/fmt-src/include")
set_property(TARGET fmt::fmt-header-only APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS FMT_HEADER_ONLY=1)
_holoscan_header_dep(dlpack::dlpack
  "${_holo_cpm}/dlpack/*/include" "${_holo_build}/_deps/dlpack-src/include")
_holoscan_header_dep(magic_enum::magic_enum
  "${_holo_cpm}/magic_enum/*/include" "${_holo_build}/_deps/magic_enum-src/include")
_holoscan_header_dep(tl::expected
  "${_holo_cpm}/tl-expected/*/include" "${_holo_build}/_deps/tl-expected-src/include")

# --- 5. Reuse the SDK's own exported imported targets ------------------------
include("${_holo_build}/holoscan-targets.cmake")

if(TARGET holoscan::core)
  message(STATUS "Holoscan SDK 5.x resolved: ${HOLOSCAN_SDK_ROOT}")
  message(STATUS "Holoscan build tree:      ${_holo_build}")
else()
  message(FATAL_ERROR "Failed to import holoscan::core from ${_holo_build}/holoscan-targets.cmake")
endif()
