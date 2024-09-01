cmake_minimum_required(VERSION 3.30.2)
project(mission_control)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)

include(FetchContent)
Set(FETCHCONTENT_QUIET FALSE)

FetchContent_Declare(
  raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_TAG 5.0
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
  GIT_TAG origin/main
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

FetchContent_MakeAvailable(raylib imgui rlimgui spdlog nanoflann)

add_library(imgui ${imgui_SOURCE_DIR}/imgui.cpp ${imgui_SOURCE_DIR}/imgui_draw.cpp ${imgui_SOURCE_DIR}/imgui_widgets.cpp ${imgui_SOURCE_DIR}/imgui_tables.cpp ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR})

add_library(rlimgui ${rlimgui_SOURCE_DIR}/rlImGui.cpp)
target_include_directories(rlimgui PUBLIC ${rlimgui_SOURCE_DIR})
target_include_directories(rlimgui PRIVATE ${raylib_SOURCE_DIR}/src ${imgui_SOURCE_DIR})

add_subdirectory(src/protos)

add_executable(gcs src/gcs/gcs.cpp src/gcs/dubins/dubins.cpp)
target_link_libraries(gcs raylib imgui rlimgui ac_grpc_proto spdlog nanoflann)
target_compile_options(gcs PRIVATE -march=native)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  target_compile_options(gcs PRIVATE -Ofast)
endif()

if (APPLE)
    target_link_libraries(gcs "-framework IOKit")
    target_link_libraries(gcs "-framework Cocoa")
    target_link_libraries(gcs "-framework OpenGL")
endif()

add_executable(air src/air/air.cpp)
target_link_libraries(air ac_grpc_proto spdlog)