cmake_minimum_required(VERSION 3.30.2)
project(path_planner)

# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

FetchContent_Declare(
  raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
)

FetchContent_Declare(
  dubins 
  GIT_REPOSITORY https://github.com/AndrewWalker/Dubins-Curves.git
)

FetchContent_MakeAvailable(raylib dubins)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} raylib dubins)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)

if (APPLE)
    target_link_libraries(${PROJECT_NAME} "-framework IOKit")
    target_link_libraries(${PROJECT_NAME} "-framework Cocoa")
    target_link_libraries(${PROJECT_NAME} "-framework OpenGL")
endif()