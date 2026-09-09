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
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(Eigen3 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)


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

  target_link_libraries(test_toolchain
    Eigen3::Eigen
  )
endif()

ament_package()
