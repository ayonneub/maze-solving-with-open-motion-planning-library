cmake_minimum_required(VERSION 3.10)
project(motion_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Try to find an OMPL config first (best case)
find_package(ompl QUIET CONFIG)

# Fallback: manual paths
if(NOT ompl_FOUND)
  # Autodetect header root that contains "ompl/base/SpaceInformation.h"
  if(EXISTS "/usr/include/ompl/base/SpaceInformation.h")
    set(OMPL_INCLUDE_DIRS "/usr/include")
  elseif(EXISTS "/usr/include/ompl-1.7/ompl/base/SpaceInformation.h")
    set(OMPL_INCLUDE_DIRS "/usr/include/ompl-1.7")
  else()
    message(FATAL_ERROR "OMPL headers not found. Install libompl-dev.")
  endif()
  # Typical shared lib path on Ubuntu
  set(OMPL_LIBRARIES "/usr/lib/x86_64-linux-gnu/libompl.so")
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Only your own includes (Eigen via target, OMPL handled below)
include_directories(
  ${CMAKE_SOURCE_DIR}/include
  /usr/local/include
)

set(PNG_SOURCE_FILES src/lodepng.cpp)

# Executable: motion_planner
add_executable(${PROJECT_NAME} src/main.cpp ${PNG_SOURCE_FILES})

# Executable: motion_planner_img_parser (separate main)
add_executable(${PROJECT_NAME}_img_parser src/image_parser.cpp ${PNG_SOURCE_FILES})

# Link deps
if(ompl_FOUND)
  target_link_libraries(${PROJECT_NAME} PRIVATE ompl::ompl Eigen3::Eigen)
  target_link_libraries(${PROJECT_NAME}_img_parser PRIVATE ompl::ompl Eigen3::Eigen)
else()
  target_include_directories(${PROJECT_NAME} PRIVATE ${OMPL_INCLUDE_DIRS})
  target_include_directories(${PROJECT_NAME}_img_parser PRIVATE ${OMPL_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} PRIVATE ${OMPL_LIBRARIES} Eigen3::Eigen)
  target_link_libraries(${PROJECT_NAME}_img_parser PRIVATE ${OMPL_LIBRARIES} Eigen3::Eigen)
endif()

message(STATUS "OMPL via config: ${ompl_FOUND}")
message(STATUS "OMPL include dirs: ${OMPL_INCLUDE_DIRS}")
message(STATUS "OMPL libraries:    ${OMPL_LIBRARIES}")
