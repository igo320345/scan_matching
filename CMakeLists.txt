cmake_minimum_required(VERSION 3.8)
project(scan_matching)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nanoflann)
find_package(tf2_ros)
find_package(nav_msgs)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(executable_name scan_matching)
set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/scan_matching.cpp
)
add_library(icp SHARED
src/icp.cpp
)
add_library(utils SHARED
src/utils.cpp
)

target_include_directories(${library_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_include_directories(icp
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_include_directories(utils
PUBLIC
"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
"$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(${library_name} PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  icp
  tf2_ros::tf2_ros
  utils
)
target_link_libraries(icp PUBLIC
  Eigen3::Eigen
  nanoflann::nanoflann
  ${sensor_msgs_TARGETS}
  ${PCL_LIBRARIES}
)

target_link_libraries(utils PUBLIC
  Eigen3::Eigen
  ${geometry_msgs_TARGETS}
  tf2::tf2
)

add_executable(${executable_name}
  src/main.cpp
)
target_include_directories(${executable_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${executable_name} PRIVATE
  ${library_name}
  icp
)
install(TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS icp
  EXPORT icp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS utils
  EXPORT utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_libraries(icp)
ament_export_libraries(utils)
ament_export_dependencies(rclcpp sensor_msgs)
ament_export_targets(${library_name})
ament_export_targets(icp)
ament_export_targets(utils)
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
