# Vendored civetweb (MIT). WebSocket support is the reason it is here; TLS, CGI
# and the server-side scripting engines are all switched off since this serves
# one static page and one socket on a lab network.
add_library(scorbot_civetweb STATIC third_party/civetweb.c)
target_include_directories(scorbot_civetweb PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
target_compile_definitions(scorbot_civetweb PUBLIC
  USE_WEBSOCKET
  NO_SSL
  NO_CGI
  NO_CACHING
)
target_link_libraries(scorbot_civetweb PUBLIC Threads::Threads ${CMAKE_DL_LIBS})
# Third-party code: do not fail our build on its warnings.
target_compile_options(scorbot_civetweb PRIVATE -w)

add_executable(scorbot_webd scorbot_webd.cc)
target_link_libraries(scorbot_webd PRIVATE scorbot_link scorbot_civetweb)
target_compile_options(scorbot_webd PRIVATE -Wall -Wextra)

# Default the served directory to the source tree so the page can be edited
# without a rebuild.
target_compile_definitions(scorbot_webd PRIVATE
  SCORBOT_DEFAULT_STATIC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/static")
