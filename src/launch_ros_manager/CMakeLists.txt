cmake_minimum_required(VERSION 3.8)
project(launch_ros_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces.
rosidl_generate_interfaces(launch_ros_manager
  "srv/ShutdownNamespace.srv"
  # DEPENDENCIES builtin_interfaces
)

# Install python modules.
ament_python_install_package(${PROJECT_NAME}_py)

# Install Python executables
install(PROGRAMS
  scripts/manager_node
  DESTINATION lib/${PROJECT_NAME}
)

# Final packaging.
ament_package()
