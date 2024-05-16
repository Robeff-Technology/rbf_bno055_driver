cmake_minimum_required(VERSION 3.5)
project(rbf_bno055_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)


include_directories(include)

add_library(rbf_bno055_driver_component SHARED
  src/rbf_bno055_driver.cpp
  src/serial_port.cpp
  src/bno055.cpp
)

ament_target_dependencies(rbf_bno055_driver_component
  rclcpp
  rclcpp_components
  sensor_msgs
)

rclcpp_components_register_node(
  rbf_bno055_driver_component
  PLUGIN "rbf_bno055_driver::BNO055Driver"
  EXECUTABLE rbf_bno055_driver
)

ament_export_targets(export_rbf_bno055_driver_component)
install(TARGETS rbf_bno055_driver_component
        EXPORT export_rbf_bno055_driver_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# # Default to C++17
# set(CMAKE_CXX_STANDARD 17)

# # Find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_ros REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)
# find_package(diagnostic_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)


# # Include directories
# include_directories(include)

# # Create a library
# add_library(${PROJECT_NAME} SHARED
#     src/rbf_bno055_driver.cpp
#     src/serial_port.cpp
#     src/rbf_bno055_driver_node.cpp
# )

# # Add dependencies
# ament_target_dependencies(rbf_bno055_driver
#     "rclcpp"
#     "rclcpp_components"
#     "sensor_msgs"
#     "diagnostic_msgs"
# )

# add_executable(rbf_bno055_driver_node 

#     src/rbf_bno055_driver_node.cpp
#     src/rbf_bno055_driver.cpp
#     src/serial_port.cpp
# )

# # Install include directory
# install(DIRECTORY include/
#     DESTINATION include
# )

# # Install library
# install(TARGETS ${PROJECT_NAME}
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin
# )

# ament_package()





# cmake_minimum_required(VERSION 3.8)
# project(rbf_bno055_driver)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()
