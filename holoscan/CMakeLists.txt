cmake_minimum_required(VERSION 3.24)
project(scorbot_holoscan LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(SCORBOT_BUILD_HOLOSCAN "Build the Holoscan 5 operators and example app" ON)
option(SCORBOT_BUILD_WEBUI "Build the scorbot_webd control UI daemon" ON)

# ---------------------------------------------------------------------------
# The wire protocol header is owned by the firmware tree. Including it from
# there rather than copying it is what keeps the two sides from drifting.
# ---------------------------------------------------------------------------
set(SCORBOT_FIRMWARE_INC "${CMAKE_CURRENT_SOURCE_DIR}/../firmware/Inc")
if(NOT EXISTS "${SCORBOT_FIRMWARE_INC}/scorbot_proto.h")
  message(FATAL_ERROR "Cannot find scorbot_proto.h in ${SCORBOT_FIRMWARE_INC}")
endif()

# ---------------------------------------------------------------------------
# Optional ibverbs backend
# ---------------------------------------------------------------------------
find_path(IBVERBS_INCLUDE_DIR infiniband/verbs.h)
find_library(IBVERBS_LIBRARY NAMES ibverbs)

if(IBVERBS_INCLUDE_DIR AND IBVERBS_LIBRARY)
  set(SCORBOT_HAVE_IBVERBS ON)
  message(STATUS "ibverbs found: ${IBVERBS_LIBRARY} - ConnectX backend enabled")
else()
  set(SCORBOT_HAVE_IBVERBS OFF)
  message(WARNING "ibverbs not found - building with the socket backend only")
endif()

# ---------------------------------------------------------------------------
# scorbot_link: the comms layer. Deliberately free of Holoscan dependencies so
# it can be used stand-alone.
# ---------------------------------------------------------------------------
add_library(scorbot_link STATIC
  src/transport.cc
  src/udp_socket_transport.cc
  src/ibv_raw_transport.cc
  src/control_client.cc
  src/scorbot_link.cc
)
target_include_directories(scorbot_link PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${SCORBOT_FIRMWARE_INC}"
)
# Packet code overlays structs on byte buffers throughout. Byte-wise access is
# already used where correctness depends on it, but disabling strict aliasing
# removes the whole class of optimizer surprises this pattern invites.
target_compile_options(scorbot_link PUBLIC -fno-strict-aliasing)
target_compile_options(scorbot_link PRIVATE -Wall -Wextra)
find_package(Threads REQUIRED)
target_link_libraries(scorbot_link PUBLIC Threads::Threads)

if(SCORBOT_HAVE_IBVERBS)
  target_compile_definitions(scorbot_link PUBLIC SCORBOT_HAVE_IBVERBS)
  target_include_directories(scorbot_link PUBLIC "${IBVERBS_INCLUDE_DIR}")
  target_link_libraries(scorbot_link PUBLIC "${IBVERBS_LIBRARY}")
endif()

# ---------------------------------------------------------------------------
# Stand-alone tools. None of these link Holoscan, which is the proof that the
# comms layer really is independent of it.
# ---------------------------------------------------------------------------
add_executable(scorbot_sim tools/scorbot_sim.cc)
target_link_libraries(scorbot_sim PRIVATE scorbot_link)
target_compile_options(scorbot_sim PRIVATE -Wall -Wextra)

add_executable(scorbot_cli tools/scorbot_cli.cc)
target_link_libraries(scorbot_cli PRIVATE scorbot_link)
target_compile_options(scorbot_cli PRIVATE -Wall -Wextra)

add_executable(scorbot_ping tools/scorbot_ping.cc)
target_link_libraries(scorbot_ping PRIVATE scorbot_link)
target_compile_options(scorbot_ping PRIVATE -Wall -Wextra)

# ---------------------------------------------------------------------------
# Web UI daemon
# ---------------------------------------------------------------------------
if(SCORBOT_BUILD_WEBUI)
  add_subdirectory(webui)
endif()

# ---------------------------------------------------------------------------
# Holoscan 5 operators and example application
# ---------------------------------------------------------------------------
if(SCORBOT_BUILD_HOLOSCAN)
  include(cmake/FindOrConfigureHoloscan.cmake)
  if(TARGET holoscan::core)
    add_subdirectory(apps)
  else()
    message(WARNING "Holoscan 5 SDK not found - skipping operators and apps. "
                    "Pass -DHOLOSCAN_INSTALL_ROOT=/path/to/holoscan to enable them.")
  endif()
endif()

add_executable(scorbot_ibvprobe tools/scorbot_ibvprobe.cc)
target_link_libraries(scorbot_ibvprobe PRIVATE scorbot_link)
target_compile_options(scorbot_ibvprobe PRIVATE -Wall -Wextra)
