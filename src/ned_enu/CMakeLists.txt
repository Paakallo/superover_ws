cmake_minimum_required(VERSION 3.8)
project(ned_enu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# includes directories in order to find below packages
INCLUDE_DIRECTORIES ("/usr/include/eigen3")
INCLUDE_DIRECTORIES ("~/sese_ws/build/px4_msgs/rosidl_generator_cpp")
include_directories(include)

# find dependecies of executable
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# find dependencies of frame_transforms
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(LIB_DEPEN
  rclcpp
  px4_msgs
  nav_msgs
  Eigen3
  geometry_msgs
  sensor_msgs
)

set(SRC_CPP src/odom_conversion.cpp)


add_library(frame_transforms SHARED src/frame_transforms.cpp)
ament_target_dependencies(frame_transforms Eigen3 geometry_msgs sensor_msgs)

target_include_directories(frame_transforms PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)


add_library(lib_odom src/odom_conversion.cpp)
ament_target_dependencies(lib_odom ${LIB_DEPEN})
include_directories(include/${PROJECT_NAME})


add_executable(odom_conversion ${SRC_CPP} src/frame_transforms.cpp)
ament_target_dependencies(odom_conversion ${LIB_DEPEN})

target_link_libraries(odom_conversion lib_odom frame_transforms)
target_include_directories(odom_conversion
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)



install(TARGETS
odom_conversion
DESTINATION lib/${PROJECT_NAME})

install(TARGETS
    frame_transforms
    lib_odom
    DESTINATION lib/${PROJECT_NAME})
# ament_package()








add_executable(enu_pub src/enu_publisher.cpp)
ament_target_dependencies(enu_pub rclcpp std_msgs)

install(TARGETS
  enu_pub
  DESTINATION lib/${PROJECT_NAME})

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
