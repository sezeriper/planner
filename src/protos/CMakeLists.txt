# Another way is to use CMake's FetchContent module to clone gRPC at
# configure time. This makes gRPC's source code available to your project,
# similar to a git submodule.
message(STATUS "Using gRPC via add_subdirectory (FetchContent).")
include(FetchContent)
FetchContent_Declare(
    grpc
    GIT_REPOSITORY https://github.com/grpc/grpc.git
    # when using gRPC, you will actually set this to an existing tag, such as
    # v1.25.0, v1.26.0 etc..
    # For the purpose of testing, we override the tag used to the commit
    # that's currently under test.
    GIT_TAG        v1.66.0
    GIT_PROGRESS   TRUE)

set(ABSL_PROPAGATE_CXX_STD ON)
set(ABSL_ENABLE_INSTALL ON)

set(gRPC_BUILD_TESTS OFF)
set(gRPC_BUILD_CODEGEN ON) # for grpc_cpp_plugin
set(gRPC_BUILD_GRPC_CPP_PLUGIN ON) # we want to use only C++ plugin
set(gRPC_BUILD_CSHARP_EXT OFF)
set(gRPC_BUILD_GRPC_CSHARP_PLUGIN OFF)
set(gRPC_BUILD_GRPC_NODE_PLUGIN OFF)
set(gRPC_BUILD_GRPC_OBJECTIVE_C_PLUGIN OFF)
set(gRPC_BUILD_GRPC_PHP_PLUGIN OFF)
set(gRPC_BUILD_GRPC_PYTHON_PLUGIN OFF)
set(gRPC_BUILD_GRPC_RUBY_PLUGIN OFF)

FetchContent_MakeAvailable(grpc)

# Since FetchContent uses add_subdirectory under the hood, we can use
# the grpc targets directly from this build.
set(_PROTOBUF_LIBPROTOBUF libprotobuf)
set(_REFLECTION grpc++_reflection)
set(_PROTOBUF_PROTOC $<TARGET_FILE:protoc>)
set(_GRPC_GRPCPP grpc++)
if(CMAKE_CROSSCOMPILING)
    find_program(_GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)
else()
    set(_GRPC_CPP_PLUGIN_EXECUTABLE $<TARGET_FILE:grpc_cpp_plugin>)
endif()

# Proto file
get_filename_component(ac_proto "air_control.proto" ABSOLUTE)
get_filename_component(ac_proto_path "${ac_proto}" PATH)

# Generated sources
set(ac_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/air_control.pb.cc")
set(ac_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/air_control.pb.h")
set(ac_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/air_control.grpc.pb.cc")
set(ac_grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/air_control.grpc.pb.h")
add_custom_command(
      OUTPUT "${ac_proto_srcs}" "${ac_proto_hdrs}" "${ac_grpc_srcs}" "${ac_grpc_hdrs}"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
        --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
        -I "${ac_proto_path}"
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${ac_proto}"
      DEPENDS "${ac_proto}")

add_library(ac_grpc_proto
  ${ac_grpc_srcs}
  ${ac_grpc_hdrs}
  ${ac_proto_srcs}
  ${ac_proto_hdrs})

# Include generated *.pb.h files
target_include_directories(ac_grpc_proto PUBLIC "${CMAKE_CURRENT_BINARY_DIR}")

target_link_libraries(ac_grpc_proto
    absl::absl_log
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF})