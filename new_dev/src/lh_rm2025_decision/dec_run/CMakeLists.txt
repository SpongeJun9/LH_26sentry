cmake_minimum_required(VERSION 3.10)
project(dec_run)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########
link_libraries(jsoncpp)
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/dec_run_node.cpp
  src/test_nav_node.cpp
  src/posture_test_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN dr::DecRunNode
  EXECUTABLE ${PROJECT_NAME}_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN dr::TestNavNode
  EXECUTABLE test_nav_node
)

ament_auto_add_executable(posture_test_node
  src/posture_test_node.cpp
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
