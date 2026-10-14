cmake_minimum_required(VERSION 3.16)
project(rco_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion
    -Wsign-conversion
    -Wshadow
    -Wnon-virtual-dtor
    -Wold-style-cast
    -Woverloaded-virtual
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCASCADE REQUIRED COMPONENTS
  FoundationClasses
  ModelingData
  ModelingAlgorithms
  Visualization
  ApplicationFramework
  DataExchange
)
find_package(yaml-cpp REQUIRED)

add_library(rco_core
  src/robot_model_context.cpp
  src/domain/csv.cpp
  src/domain/serialization.cpp
  src/domain/validation.cpp
  src/reachability/result.cpp
  src/reachability/seed.cpp
  src/reachability/ik.cpp
  src/reachability/path.cpp
  src/task/path.cpp
  src/task/visualization.cpp
  src/scene/collision_object.cpp
  src/scene/planning_scene.cpp
  src/scene/step_mesh.cpp
)

target_include_directories(rco_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(rco_core SYSTEM
  PRIVATE
    ${OpenCASCADE_INCLUDE_DIR}
)

ament_target_dependencies(rco_core
  ament_index_cpp
  rclcpp
  geometric_shapes
  geometry_msgs
  sensor_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  shape_msgs
  visualization_msgs
)

target_link_libraries(rco_core
  Eigen3::Eigen
  yaml-cpp::yaml-cpp
  TKMesh
  TKSTEP
)

add_executable(robot_diagnostics
  src/robot_diagnostics.cpp
)

target_link_libraries(robot_diagnostics
  rco_core
)

ament_target_dependencies(robot_diagnostics
  rclcpp
)

install(
  TARGETS rco_core
  EXPORT export_rco_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS robot_diagnostics
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_rco_core HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
  rclcpp
  geometric_shapes
  geometry_msgs
  sensor_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  shape_msgs
  visualization_msgs
  Eigen3
  yaml-cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(rco_description REQUIRED)

  # Project policy:
  # - ClangFormat is the canonical C++ formatter.
  # - ClangTidy is the canonical C++ static-analysis tool.
  # - Do not run competing/default C++ style checkers through ament_lint_common.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_cppcheck
    ament_cmake_uncrustify
  )

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_toolchain
    test/test_toolchain.cpp
  )

  ament_add_gtest(test_domain_validation
    test/domain/test_validation.cpp
  )

  target_link_libraries(test_domain_validation
    rco_core
  )

  ament_add_gtest(test_domain_serialization
    test/domain/test_serialization.cpp
  )

  target_link_libraries(test_domain_serialization
    rco_core
  )

  ament_add_gtest(test_domain_csv
    test/domain/test_csv.cpp
  )

  target_link_libraries(test_domain_csv
    rco_core
  )

  ament_add_gtest(test_reachability_result
    test/reachability/test_result.cpp
  )

  target_link_libraries(test_reachability_result
    rco_core
  )

  ament_add_gtest(test_reachability_seed
    test/reachability/test_seed.cpp
  )

  target_link_libraries(test_reachability_seed
    rco_core
  )

  ament_add_gtest(test_task_path
    test/task/test_path.cpp
  )

  target_link_libraries(test_task_path
    rco_core
  )

  ament_add_gtest(test_task_visualization
    test/task/test_visualization.cpp
  )

  target_link_libraries(test_task_visualization
    rco_core
  )

  ament_add_gtest(test_collision_object
    test/scene/test_collision_object.cpp
  )

  target_link_libraries(test_collision_object
    rco_core
    TKPrim
    TKSTEP
  )

  target_include_directories(test_collision_object SYSTEM
    PRIVATE
      ${OpenCASCADE_INCLUDE_DIR}
  )

  ament_add_gtest(test_planning_scene
    test/scene/test_planning_scene.cpp
  )

  target_link_libraries(test_planning_scene
    rco_core
    moveit_core::moveit_test_utils
  )

  target_link_libraries(test_toolchain
    Eigen3::Eigen
  )

  ament_add_gtest(test_fk_ik_roundtrip
    test/test_fk_ik_roundtrip.cpp
    SKIP_TEST
  )

  target_link_libraries(test_fk_ik_roundtrip
    rco_core
    Eigen3::Eigen
  )

  ament_target_dependencies(test_fk_ik_roundtrip
    rclcpp
    moveit_core
    moveit_ros_planning
  )


  install(
    TARGETS test_fk_ik_roundtrip
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  add_launch_test(
    test/test_fk_ik_launch.py
    TIMEOUT 60
  )

  ament_add_gtest(test_robot_model_context
    test/test_robot_model_context.cpp
    test/reachability/test_ik.cpp
    SKIP_TEST
  )

  target_link_libraries(test_robot_model_context
    rco_core
    Eigen3::Eigen
  )

  ament_target_dependencies(test_robot_model_context
    rclcpp
    moveit_core
    moveit_ros_planning
  )

  install(
    TARGETS test_robot_model_context
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  add_launch_test(
    test/test_robot_model_context_launch.py
    TIMEOUT 60
  )


endif()

ament_package()
