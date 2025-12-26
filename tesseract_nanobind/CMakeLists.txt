cmake_minimum_required(VERSION 3.15)
project(tesseract_nanobind LANGUAGES CXX)

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

# Set RPATH to find tesseract libraries at runtime
# This embeds the full path to ws/install/lib in the built .so files
set(CMAKE_INSTALL_RPATH "${TESSERACT_INSTALL_DIR}/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
if(APPLE)
  set(CMAKE_MACOSX_RPATH ON)
  # Also add conda lib for dependencies like boost, tinyxml2
  if(DEFINED ENV{CONDA_PREFIX})
    list(APPEND CMAKE_INSTALL_RPATH "$ENV{CONDA_PREFIX}/lib")
  endif()
endif()

# Add conda prefix for dependencies (tinyxml2, console_bridge, etc.)
if(DEFINED ENV{CONDA_PREFIX})
  list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
  link_directories("$ENV{CONDA_PREFIX}/lib")
  # Qt6 requires QT_HOST_PATH on macOS with conda
  set(QT_HOST_PATH "$ENV{CONDA_PREFIX}" CACHE PATH "Qt host path for cross-compilation")
  message(STATUS "Added conda prefix: $ENV{CONDA_PREFIX}")
endif()

# Find tesseract C++ libraries
find_package(tesseract_common REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_srdf REQUIRED)
find_package(tesseract_urdf REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_state_solver REQUIRED)
find_package(tesseract_kinematics REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_command_language REQUIRED)
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

  # RTTI visibility fix: Export typeinfo symbols so typeid() comparison works
  # across library boundaries (needed for TaskComposer plugin compatibility)
  set_target_properties(_${module_name} PROPERTIES
    CXX_VISIBILITY_PRESET default
    VISIBILITY_INLINES_HIDDEN OFF)

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

# Add tesseract_srdf module
add_tesseract_nanobind_extension(
  tesseract_srdf
  src/tesseract_srdf/tesseract_srdf_bindings.cpp
  tesseract::tesseract_srdf
  tesseract::tesseract_scene_graph
  tesseract::tesseract_common
)

# Add tesseract_urdf module
add_tesseract_nanobind_extension(
  tesseract_urdf
  src/tesseract_urdf/tesseract_urdf_bindings.cpp
  tesseract::tesseract_urdf
  tesseract::tesseract_scene_graph
  tesseract::tesseract_common
)

# Add tesseract_collision module
add_tesseract_nanobind_extension(
  tesseract_collision
  src/tesseract_collision/tesseract_collision_bindings.cpp
  tesseract::tesseract_collision_core
  tesseract::tesseract_collision_bullet
  tesseract::tesseract_geometry
  tesseract::tesseract_common
)

# Add tesseract_state_solver module
add_tesseract_nanobind_extension(
  tesseract_state_solver
  src/tesseract_state_solver/tesseract_state_solver_bindings.cpp
  tesseract::tesseract_state_solver_kdl
  tesseract::tesseract_state_solver_ofkt
  tesseract::tesseract_scene_graph
  tesseract::tesseract_common
)

# Add tesseract_kinematics module
add_tesseract_nanobind_extension(
  tesseract_kinematics
  src/tesseract_kinematics/tesseract_kinematics_bindings.cpp
  tesseract::tesseract_kinematics_core
  tesseract::tesseract_scene_graph
  tesseract::tesseract_state_solver_kdl
  tesseract::tesseract_common
)

# Add tesseract_environment module
add_tesseract_nanobind_extension(
  tesseract_environment
  src/tesseract_environment/tesseract_environment_bindings.cpp
  tesseract::tesseract_environment
  tesseract::tesseract_kinematics_core
  tesseract::tesseract_collision_core
  tesseract::tesseract_srdf
  tesseract::tesseract_scene_graph
  tesseract::tesseract_common
)

# Add tesseract_command_language module
add_tesseract_nanobind_extension(
  tesseract_command_language
  src/tesseract_command_language/tesseract_command_language_bindings.cpp
  tesseract::tesseract_command_language
  tesseract::tesseract_common
)

# Find motion planners
find_package(tesseract_motion_planners REQUIRED COMPONENTS core simple ompl)

# Add tesseract_motion_planners module
add_tesseract_nanobind_extension(
  tesseract_motion_planners
  src/tesseract_motion_planners/tesseract_motion_planners_bindings.cpp
  tesseract::tesseract_motion_planners_core
  tesseract::tesseract_environment
  tesseract::tesseract_command_language
  tesseract::tesseract_common
)

# Add tesseract_motion_planners_simple module
add_tesseract_nanobind_extension(
  tesseract_motion_planners_simple
  src/tesseract_motion_planners_simple/tesseract_motion_planners_simple_bindings.cpp
  tesseract::tesseract_motion_planners_simple
  tesseract::tesseract_motion_planners_core
  tesseract::tesseract_environment
  tesseract::tesseract_command_language
  tesseract::tesseract_common
)

# Add tesseract_motion_planners_ompl module
add_tesseract_nanobind_extension(
  tesseract_motion_planners_ompl
  src/tesseract_motion_planners_ompl/tesseract_motion_planners_ompl_bindings.cpp
  tesseract::tesseract_motion_planners_ompl
  tesseract::tesseract_motion_planners_core
  tesseract::tesseract_environment
  tesseract::tesseract_command_language
  tesseract::tesseract_common
)

# Add tesseract_motion_planners_descartes module (optional - only if descartes component was built)
find_package(tesseract_motion_planners QUIET COMPONENTS descartes)
find_package(descartes_light QUIET)
if(tesseract_motion_planners_descartes_FOUND AND descartes_light_FOUND)
  message(STATUS "tesseract_motion_planners descartes component found - building bindings")
  add_tesseract_nanobind_extension(
    tesseract_motion_planners_descartes
    src/tesseract_motion_planners_descartes/tesseract_motion_planners_descartes_bindings.cpp
    tesseract::tesseract_motion_planners_descartes
    tesseract::tesseract_motion_planners_core
    tesseract::tesseract_environment
    tesseract::tesseract_command_language
    tesseract::tesseract_collision_core
    tesseract::tesseract_common
  )
else()
  message(STATUS "tesseract_motion_planners descartes component not found - skipping bindings")
endif()

# Add tesseract_motion_planners_trajopt module (optional - only if trajopt component was built)
find_package(tesseract_motion_planners QUIET COMPONENTS trajopt)
find_package(trajopt QUIET)
if(tesseract_motion_planners_trajopt_FOUND AND trajopt_FOUND)
  message(STATUS "tesseract_motion_planners trajopt component found - building bindings")
  add_tesseract_nanobind_extension(
    tesseract_motion_planners_trajopt
    src/tesseract_motion_planners_trajopt/tesseract_motion_planners_trajopt_bindings.cpp
    tesseract::tesseract_motion_planners_trajopt
    tesseract::tesseract_motion_planners_core
    tesseract::tesseract_environment
    tesseract::tesseract_command_language
    tesseract::tesseract_common
    trajopt::trajopt
  )
else()
  message(STATUS "tesseract_motion_planners trajopt component not found - skipping bindings")
endif()

# Add tesseract_motion_planners_trajopt_ifopt module (optional - only if trajopt_ifopt component was built)
find_package(tesseract_motion_planners QUIET COMPONENTS trajopt_ifopt)
find_package(trajopt_ifopt QUIET)
find_package(trajopt_sqp QUIET)
if(tesseract_motion_planners_trajopt_ifopt_FOUND AND trajopt_ifopt_FOUND AND trajopt_sqp_FOUND)
  message(STATUS "tesseract_motion_planners trajopt_ifopt component found - building bindings")
  add_tesseract_nanobind_extension(
    tesseract_motion_planners_trajopt_ifopt
    src/tesseract_motion_planners_trajopt_ifopt/tesseract_motion_planners_trajopt_ifopt_bindings.cpp
    tesseract::tesseract_motion_planners_trajopt_ifopt
    tesseract::tesseract_motion_planners_core
    tesseract::tesseract_environment
    tesseract::tesseract_command_language
    tesseract::tesseract_common
    trajopt::trajopt_ifopt
    trajopt::trajopt_sqp
  )
else()
  message(STATUS "tesseract_motion_planners trajopt_ifopt component not found - skipping bindings")
endif()

# Find and add tesseract_time_parameterization
find_package(tesseract_time_parameterization REQUIRED)

add_tesseract_nanobind_extension(
  tesseract_time_parameterization
  src/tesseract_time_parameterization/tesseract_time_parameterization_bindings.cpp
  tesseract::tesseract_time_parameterization_isp
  tesseract::tesseract_time_parameterization_totg
  tesseract::tesseract_time_parameterization_core
  tesseract::tesseract_command_language
  tesseract::tesseract_common
)

# Find and add tesseract_task_composer (core and taskflow components only, planning requires trajopt)
find_package(tesseract_task_composer REQUIRED COMPONENTS core taskflow)

add_tesseract_nanobind_extension(
  tesseract_task_composer
  src/tesseract_task_composer/tesseract_task_composer_bindings.cpp
  tesseract::tesseract_task_composer
  tesseract::tesseract_task_composer_taskflow
  tesseract::tesseract_environment
  tesseract::tesseract_command_language
  tesseract::tesseract_common
)

message(STATUS "============= Build Configuration =============")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "Python: ${Python_VERSION}")
message(STATUS "nanobind: ${nanobind_VERSION}")
message(STATUS "===============================================")

# ============== Install Bundled Data Files ==============
# These are installed into the wheel so env vars can use relative paths
# Support both local dev (../ws/src/...) and CI (../../tesseract/...) layouts

set(TESSERACT_SUPPORT_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../ws/src/tesseract/tesseract_support")
if(NOT EXISTS "${TESSERACT_SUPPORT_SRC}")
  set(TESSERACT_SUPPORT_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../../tesseract/tesseract_support")
endif()

set(TASK_COMPOSER_CFG_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../ws/src/tesseract_planning/tesseract_task_composer/config")
if(NOT EXISTS "${TASK_COMPOSER_CFG_SRC}")
  set(TASK_COMPOSER_CFG_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../../tesseract_planning/tesseract_task_composer/config")
endif()

if(EXISTS "${TESSERACT_SUPPORT_SRC}")
  message(STATUS "Installing tesseract_support from: ${TESSERACT_SUPPORT_SRC}")
  install(DIRECTORY "${TESSERACT_SUPPORT_SRC}/"
    DESTINATION tesseract_robotics/data/tesseract_support
    FILES_MATCHING
    PATTERN "*.urdf" PATTERN "*.srdf" PATTERN "*.yaml" PATTERN "*.xacro"
    PATTERN "*.stl" PATTERN "*.STL" PATTERN "*.dae" PATTERN "*.ply" PATTERN "*.obj" PATTERN "*.csv"
    PATTERN "*.xml" PATTERN "*.png" PATTERN "*.jpg" PATTERN "*.jpeg"
    PATTERN ".git" EXCLUDE PATTERN "__pycache__" EXCLUDE)
else()
  message(WARNING "tesseract_support not found at ${TESSERACT_SUPPORT_SRC}")
endif()

if(EXISTS "${TASK_COMPOSER_CFG_SRC}")
  message(STATUS "Installing task_composer config from: ${TASK_COMPOSER_CFG_SRC}")
  install(DIRECTORY "${TASK_COMPOSER_CFG_SRC}/"
    DESTINATION tesseract_robotics/data/task_composer_config
    FILES_MATCHING PATTERN "*.yaml" PATTERN "*.schema")
else()
  message(WARNING "task_composer config not found at ${TASK_COMPOSER_CFG_SRC}")
endif()
