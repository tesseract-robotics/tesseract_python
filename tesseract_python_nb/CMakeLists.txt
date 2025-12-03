cmake_minimum_required(VERSION 3.15)
project(tesseract_python_nb LANGUAGES CXX)

# Build configuration
set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find Python and nanobind
find_package(Python 3.9 REQUIRED COMPONENTS Interpreter Development.Module Development.SABIModule)
find_package(nanobind CONFIG REQUIRED)

# Add tesseract install path to CMAKE_PREFIX_PATH
# Use absolute path derived from project source dir
get_filename_component(TESSERACT_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ws/install" ABSOLUTE)
list(APPEND CMAKE_PREFIX_PATH "${TESSERACT_INSTALL_DIR}")
message(STATUS "Added tesseract install dir: ${TESSERACT_INSTALL_DIR}")

# Add conda prefix for dependencies (tinyxml2, console_bridge, etc.)
if(DEFINED ENV{CONDA_PREFIX})
  list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
  link_directories("$ENV{CONDA_PREFIX}/lib")
  message(STATUS "Added conda prefix: $ENV{CONDA_PREFIX}")
endif()

# Find tesseract C++ libraries
find_package(tesseract_common REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(console_bridge REQUIRED)

# Define reusable function for nanobind extensions
function(add_tesseract_nanobind_extension module_name source_file)
  nanobind_add_module(
    _${module_name}
    STABLE_ABI
    NB_STATIC
    ${source_file}
  )

  # Apply precompiled headers
  target_precompile_headers(_${module_name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tesseract_nb.h)

  # Include directories
  target_include_directories(_${module_name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # Link tesseract libraries (passed as remaining arguments)
  target_link_libraries(_${module_name} PRIVATE ${ARGN})

  # Install the module
  install(TARGETS _${module_name}
          LIBRARY DESTINATION tesseract_robotics/${module_name})
endfunction()

# Add tesseract_common module
add_tesseract_nanobind_extension(
  tesseract_common
  src/tesseract_common/tesseract_common_bindings.cpp
  tesseract::tesseract_common
  console_bridge::console_bridge
)

# Add tesseract_geometry module
add_tesseract_nanobind_extension(
  tesseract_geometry
  src/tesseract_geometry/tesseract_geometry_bindings.cpp
  tesseract::tesseract_geometry
  tesseract::tesseract_common
)

# Add tesseract_scene_graph module
add_tesseract_nanobind_extension(
  tesseract_scene_graph
  src/tesseract_scene_graph/tesseract_scene_graph_bindings.cpp
  tesseract::tesseract_scene_graph
  tesseract::tesseract_geometry
  tesseract::tesseract_common
)

message(STATUS "============= Build Configuration =============")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "Python: ${Python_VERSION}")
message(STATUS "nanobind: ${nanobind_VERSION}")
message(STATUS "===============================================")
