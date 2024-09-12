cmake_minimum_required(VERSION 3.28.2)
project(mission_control)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED 20)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -stdlib=libc++")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -march=native -stdlib=libc++ -lc++abi")

include(FetchContent)
Set(FETCHCONTENT_QUIET FALSE)

FetchContent_Declare(
  raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_TAG 10b01ba7c2139b9203045bb56aa8403ca810cc6b
  GIT_PROGRESS TRUE
)

FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.91.0-docking
  GIT_PROGRESS TRUE
)

FetchContent_Declare(
  rlimgui
  GIT_REPOSITORY https://github.com/raylib-extras/rlImGui.git
  GIT_TAG 57efef06f69b50ed9ac0834a963a8aefcc59d588
  GIT_PROGRESS TRUE
)

FetchContent_Declare(
  spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG v1.14.1
  GIT_PROGRESS TRUE
)

set(MASTER_PROJECT_HAS_TARGET_UNINSTALL ON)
FetchContent_Declare(
  nanoflann
  GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git
  GIT_TAG v1.6.1
  GIT_PROGRESS TRUE
)

# FetchContent_Declare(
#   mavlink
#   GIT_REPOSITORY https://github.com/mavlink/mavlink.git
#   GIT_TAG f1d42e2774cae767a1c0651b0f95e3286c587257
#   GIT_PROGRESS TRUE
# )

set(SUPERBUILD ON)
set(BUILD_SHARED_LIBSS OFF)
set(BUILD_TESTS OFF)
set(BUILD_TESTING OFF)
set(BUILD_MAVSDK_SERVER OFF)

FetchContent_Declare(
  mavsdk
  GIT_REPOSITORY https://github.com/mavlink/MAVSDK.git
  GIT_TAG v2.12.5
  GIT_PROGRESS TRUE
)

set(CURL_ZLIB OFF)
FetchContent_Declare(
  cpr
  GIT_REPOSITORY https://github.com/libcpr/cpr.git
  GIT_TAG 1.10.5
  GIT_PROGRESS TRUE
)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)


FetchContent_MakeAvailable(raylib imgui rlimgui spdlog nanoflann mavsdk cpr json)
# FetchContent_MakeAvailable(raylib imgui rlimgui spdlog nanoflann mavlink mavsdk)

add_library(imgui ${imgui_SOURCE_DIR}/imgui.cpp ${imgui_SOURCE_DIR}/imgui_draw.cpp ${imgui_SOURCE_DIR}/imgui_widgets.cpp ${imgui_SOURCE_DIR}/imgui_tables.cpp ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR})

add_library(rlimgui ${rlimgui_SOURCE_DIR}/rlImGui.cpp)
target_include_directories(rlimgui PUBLIC ${rlimgui_SOURCE_DIR})
target_include_directories(rlimgui PRIVATE ${raylib_SOURCE_DIR}/src ${imgui_SOURCE_DIR})

add_subdirectory(src/protos)

add_executable(gcs src/gcs/gcs.cpp src/dubins/dubins.cpp)
target_link_libraries(gcs raylib imgui rlimgui ac_grpc_proto spdlog nanoflann mavsdk cpr nlohmann_json)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  target_compile_options(gcs PRIVATE -Ofast)
endif()

if (APPLE)
    target_link_libraries(gcs "-framework IOKit")
    target_link_libraries(gcs "-framework Cocoa")
    target_link_libraries(gcs "-framework OpenGL")
endif()

file(COPY resources DESTINATION ${CMAKE_BINARY_DIR})

add_executable(air src/air/air.cpp src/dubins/dubins.cpp)
target_link_libraries(air ac_grpc_proto spdlog mavsdk)