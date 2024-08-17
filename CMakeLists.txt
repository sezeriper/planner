cmake_minimum_required(VERSION 3.30.2)
project(path_planner)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
Set(FETCHCONTENT_QUIET FALSE)

FetchContent_Declare(
  raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_PROGRESS TRUE
)

FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG origin/docking
  GIT_PROGRESS TRUE
)

FetchContent_Declare(
  rlimgui
  GIT_REPOSITORY https://github.com/raylib-extras/rlImGui.git
  GIT_TAG origin/main
  GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(raylib imgui rlimgui)

add_library(imgui ${imgui_SOURCE_DIR}/imgui.cpp ${imgui_SOURCE_DIR}/imgui_draw.cpp ${imgui_SOURCE_DIR}/imgui_widgets.cpp ${imgui_SOURCE_DIR}/imgui_tables.cpp ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR})
target_compile_features(imgui PRIVATE cxx_std_20)

add_library(rlimgui ${rlimgui_SOURCE_DIR}/rlImGui.cpp)
target_include_directories(rlimgui PUBLIC ${rlimgui_SOURCE_DIR})
target_include_directories(rlimgui PRIVATE ${raylib_SOURCE_DIR}/src ${imgui_SOURCE_DIR})
target_compile_features(rlimgui PRIVATE cxx_std_20)

add_executable(${PROJECT_NAME} src/main.cpp src/dubins/dubins.cpp)
target_link_libraries(${PROJECT_NAME} raylib imgui rlimgui)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)

if (APPLE)
    target_link_libraries(${PROJECT_NAME} "-framework IOKit")
    target_link_libraries(${PROJECT_NAME} "-framework Cocoa")
    target_link_libraries(${PROJECT_NAME} "-framework OpenGL")
endif()