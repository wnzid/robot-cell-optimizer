cmake_minimum_required(VERSION 3.16)
project(rco_description VERSION 0.1.0)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
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
endif()

ament_package()
