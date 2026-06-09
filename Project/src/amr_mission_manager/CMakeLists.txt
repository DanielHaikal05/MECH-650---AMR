cmake_minimum_required(VERSION 3.8)
project(amr_mission_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteMission.action"
  "srv/GetLandmarkPosition.srv"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs
)

# NOTE: ament_python_install_package(${PROJECT_NAME}) is intentionally
# absent because rosidl_generate_interfaces above auto-creates the same
# target name (ament_cmake_python_build_<project>_egg) for its
# auto-generated Python bindings. Phase 9 has no manual Python source
# in this package; rosidl owns amr_mission_manager.* in the install tree.
# Phase 8 will add Python server source — that phase needs to either
# (a) put server source in a separate package, or (b) inline the server
# source via install(PROGRAMS ...) bypassing ament_python_install_package,
# or (c) accept the convention where rosidl OWNS the Python package and
# the server source lives at amr_mission_manager/amr_mission_manager/
# AFTER moving the manual __init__.py out of the way. Phase 8 brief
# will resolve this.

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
