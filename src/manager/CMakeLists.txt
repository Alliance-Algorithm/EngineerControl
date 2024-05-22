cmake_minimum_required(VERSION 3.8)
project(manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

install(
  DIRECTORY YuHengArm
  DESTINATION share/${PROJECT_NAME}
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE WORLD_EXECUTE)
  
find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies (REQUIRED)


set(PROJECT_SOURCES
    src/main.cpp
)

ament_auto_add_executable (
    ${PROJECT_NAME}
    ${PROJECT_SOURCES}
)

# Headers
include_directories(include src)

ament_auto_package()

