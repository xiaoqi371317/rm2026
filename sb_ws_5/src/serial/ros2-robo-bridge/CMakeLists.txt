cmake_minimum_required(VERSION 3.8)
project(robo_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/robo_bridge_node.cpp
  src/robo_bridge.cpp
  src/my_listener.cpp
)

ament_target_dependencies(${PROJECT_NAME} std_msgs rclcpp robo_utils)
target_include_directories(${PROJECT_NAME} PUBLIC 3rdparty/include)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(${PROJECT_NAME} PUBLIC 3rdparty/lib/amd64)
  install(
    DIRECTORY 3rdparty/lib/amd64/
    DESTINATION lib
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(${PROJECT_NAME} PUBLIC hikSDK/lib/arm64)
  install(
    DIRECTORY 3rdparty/lib/arm64/
    DESTINATION lib
  )
else()
  message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

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

target_link_libraries(${PROJECT_NAME}
  cserialport
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN robo_bridge::RoboBridgeNode
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)
