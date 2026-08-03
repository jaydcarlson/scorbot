add_executable(scorbot_stream scorbot_stream.cc)
target_compile_features(scorbot_stream PRIVATE cxx_std_20)
target_include_directories(scorbot_stream PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/..")
target_link_libraries(scorbot_stream PRIVATE scorbot_link holoscan::core)
# The SDK's option structs are designed for designated initializers, so partial
# initialization is intentional rather than an oversight.
target_compile_options(scorbot_stream PRIVATE -Wall -Wextra -Wno-missing-field-initializers)

# The SDK build tree is not installed, so bake its library directory into the
# RPATH rather than making callers set LD_LIBRARY_PATH.
if(HOLOSCAN_LIB_DIR)
  set_target_properties(scorbot_stream PROPERTIES
    BUILD_RPATH "${HOLOSCAN_LIB_DIR}"
    INSTALL_RPATH "${HOLOSCAN_LIB_DIR}")
endif()
