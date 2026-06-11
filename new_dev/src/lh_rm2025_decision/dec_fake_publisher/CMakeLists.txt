cmake_minimum_required(VERSION 3.8)
project(dec_fake_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(core_global REQUIRED)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()


add_executable(dec_fake_publisher_node src/dec_fake_publisher_node.cpp)

               
ament_target_dependencies(dec_fake_publisher_node #添加功能包依赖
  rclcpp 
  std_msgs
  core_global
) 
install(
  TARGETS dec_fake_publisher_node
  DESTINATION lib/${PROJECT_NAME})


