cmake_minimum_required(VERSION 3.5)

project(lex_node)


# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lex_common_msgs REQUIRED)
find_package(lex_common REQUIRED)
find_package(aws_common REQUIRED)
find_package(aws_ros2_common REQUIRED)

if (AWSSDK_FOUND)
    set(SERVICE lex)
    AWSSDK_DETERMINE_LIBS_TO_LINK(SERVICE OUTPUT)
    link_directories("${AWSSDK_LIB_DIR}")
endif ()

add_definitions(-DUSE_IMPORT_EXPORT)

set(LEX_LIBRARY_TARGET ${PROJECT_NAME}_lib)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${rclcpp_INCLUDE_DIRS}
        ${aws_ros2_common_INCLUDE_DIRS}
        ${AWSSDK_INCLUDE_DIRS}
        ${aws_common_INCLUDE_DIRS}
        ${lex_common_INCLUDE_DIRS}
)

add_library(${LEX_LIBRARY_TARGET}
        src/lex_node.cpp
        )

ament_target_dependencies(${LEX_LIBRARY_TARGET}
        rclcpp
        lex_common
        lex_common_msgs
        aws_common
        rosidl_typesupport_cpp
        aws_ros2_common
        )

target_link_libraries(${LEX_LIBRARY_TARGET}
        ${aws_common_LIBRARIES}
        ${rclcpp_LIBRARIES}
        ${aws_common_LIBRARIES}
        ${aws_ros2_common_LIBRARIES}
        ${lex_common_LIBRARIES}
        )

add_executable(${PROJECT_NAME} src/main.cpp)

ament_target_dependencies(${PROJECT_NAME}
    ${LEX_LIBRARY_TARGET}
    )

# specific order: dependents before dependencies
ament_target_dependencies(${PROJECT_NAME}
        ${LEX_LIBRARY_TARGET}
        lex_common
        )

target_link_libraries(${PROJECT_NAME} ${LEX_LIBRARY_TARGET} ${lex_common_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${LEX_LIBRARY_TARGET}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        # According to https://answers.ros.org/question/280127/ros2-ros2-run-package-executable-cannot-find-executable/
        # executables are now being installed to the library directory under project name
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

## Mark cpp header files for installation
install(
        DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
)

### Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  ament_find_gmock()
  include_directories(${GMOCK_INCLUDE_DIRS})
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies() TODO: fix linting issues.
  ament_add_gmock(test_lex_node
          test/lex_node_test.cpp
  )
  if(TARGET test_lex_node)
    target_include_directories(test_lex_node
      PRIVATE include
      PUBLIC ${GMOCK_INCLUDE_DIRS})

  target_link_libraries(test_lex_node ${LEX_LIBRARY_TARGET} ${GMOCK_LIBRARIES})
  endif()
endif()

ament_package()
