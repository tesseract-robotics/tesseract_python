cmake_minimum_required(VERSION 3.5.0...3.99)

if(${CMAKE_VERSION} VERSION_GREATER "3.27.0" OR ${CMAKE_VERSION} VERSION_EQUAL "3.27.0")
cmake_policy(SET CMP0148 OLD)
endif()

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

# Read version
FILE (STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" tesseract_python_version1 REGEX "<version>[0-9]+\\.[0-9]+\\.[0-9]+</version>")
IF ("${tesseract_python_version1}" STREQUAL "" )
MESSAGE(FATAL_ERROR "Could not read tesseract_python version.")
ENDIF()
STRING (REGEX REPLACE "<version>([0-9]+\\.[0-9]+\\.[0-9]+)</version>" "\\1" tesseract_python_version2 ${tesseract_python_version1})
STRING (STRIP ${tesseract_python_version2} tesseract_python_version)
MESSAGE(STATUS "tesseract_python version: ${tesseract_python_version}")


if (NOT MSVC)
  add_compile_options(-w)
else()
  add_compile_options(/bigobj)
endif()

find_package(tesseract REQUIRED)

find_package(tesseract_planning REQUIRED)

if(NOT TinyXML2_INCLUDE_DIRS AND TARGET tinyxml2::tinyxml2)
get_target_property(TinyXML2_INCLUDE_DIRS tinyxml2::tinyxml2 INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS "Found from target interface: TinyXML2_INCLUDE_DIRS=${TinyXML2_INCLUDE_DIRS}")
endif()

tesseract_variables()

find_bullet()

set(TARGET_INCLUDES_LIST
  trajopt::trajopt
  tesseract::common
  tesseract::geometry
  tesseract::collision
  tesseract::motion_planners
  tesseract::kinematics
  tesseract::urdf
  tesseract::scene_graph
  tesseract::srdf
  tesseract::environment
  tesseract::command_language
  tesseract::time_parameterization
  tesseract::visualization
  tesseract::task_composer
  tinyxml2::tinyxml2
)

set(TESSERACT_INCLUDE_PATHS "")

foreach(TGT IN LISTS TARGET_INCLUDES_LIST)
    if(TARGET ${TGT})
        get_target_property(INT_DIRS ${TGT} INTERFACE_INCLUDE_DIRECTORIES)
        if(INT_DIRS)
            list(APPEND TESSERACT_INCLUDE_PATHS ${INT_DIRS})
        endif()
    endif()
endforeach()

if(TESSERACT_INCLUDE_PATHS)
    list(REMOVE_DUPLICATES TESSERACT_INCLUDE_PATHS)
endif()

include_directories(${TESSERACT_INCLUDE_PATHS})

message(STATUS "TESSERACT_INCLUDE_PATHS=${TESSERACT_INCLUDE_PATHS}")

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

if(${CMAKE_VERSION} VERSION_GREATER "3.13.0" OR ${CMAKE_VERSION} VERSION_EQUAL "3.13.0")
cmake_policy(SET CMP0078 OLD)
endif()

if(${CMAKE_VERSION} VERSION_GREATER "3.14.0" OR ${CMAKE_VERSION} VERSION_EQUAL "3.14.0")
cmake_policy(SET CMP0086 OLD)
endif()

find_package(SWIG 4.1.1 REQUIRED)
include(UseSWIG)

macro(tesseract_python_find_target lib_name search_target)
  # Find install directory of search_target

  get_target_property(search_target_location ${search_target} LOCATION)
  if (NOT WIN32)
  get_filename_component(search_target_location_dir ${search_target_location} DIRECTORY)
  else()
  get_filename_component(search_target_location_dir1 ${search_target_location} DIRECTORY)
  get_filename_component(search_target_location_dir2 ${search_target_location_dir1} DIRECTORY)
  set(search_target_location_dir "${search_target_location_dir2}/lib")
  endif()
  # Find library lib_name
  unset(lib_name_location)
  find_library(lib_name_location ${lib_name} HINTS ${search_target_location_dir} NO_CACHE)
  if (NOT lib_name_location)
    message(FATAL_ERROR "Could not find library ${lib_name}")
  endif()
  # Create import target
  add_library(tesseract::${lib_name} SHARED IMPORTED)
  if (WIN32)
  set_target_properties(tesseract::${lib_name} PROPERTIES IMPORTED_IMPLIB_RELWITHDEBINFO ${lib_name_location})
  set_target_properties(tesseract::${lib_name} PROPERTIES IMPORTED_IMPLIB ${lib_name_location})
  set_target_properties(tesseract::${lib_name} PROPERTIES IMPORTED_LOCATION ${lib_name_location})
  else()
  set_target_properties(tesseract::${lib_name} PROPERTIES IMPORTED_LOCATION ${lib_name_location})
  endif()
endmacro()

tesseract_python_find_target(tesseract_collision_fcl_factories tesseract::collision)
tesseract_python_find_target(tesseract_collision_bullet_factories tesseract::collision)
tesseract_python_find_target(tesseract_kinematics_kdl_factories tesseract::kinematics)
tesseract_python_find_target(tesseract_kinematics_ur_factory tesseract::kinematics)
tesseract_python_find_target(tesseract_kinematics_opw_factory tesseract::kinematics)
#tesseract_python_find_target(tesseract_kinematics_core_factories tesseract::kinematics)
tesseract_python_find_target(tesseract_task_composer_factories tesseract::task_composer)
tesseract_python_find_target(tesseract_task_composer_planning_factories tesseract::task_composer)
tesseract_python_find_target(tesseract_task_composer_planning_nodes tesseract::task_composer)
tesseract_python_find_target(tesseract_task_composer_taskflow_factories tesseract::task_composer)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/tesseract_python_module.cmake)

tesseract_python_module(tesseract_common_python SWIG_SRCS swig/tesseract_common_python.i PACKAGE tesseract_common LIBS tesseract::common)
tesseract_python_module(tesseract_geometry_python SWIG_SRCS swig/tesseract_geometry_python.i PACKAGE tesseract_geometry LIBS tesseract::geometry )
tesseract_python_module(tesseract_collision_python SWIG_SRCS swig/tesseract_collision_python.i PACKAGE tesseract_collision LIBS tesseract::collision Bullet3::Bullet tesseract::tesseract_collision_bullet_factories tesseract::tesseract_collision_fcl_factories )
tesseract_python_module(tesseract_scene_graph_python SWIG_SRCS swig/tesseract_scene_graph_python.i PACKAGE tesseract_scene_graph LIBS tesseract::scene_graph )
tesseract_python_module(tesseract_srdf_python SWIG_SRCS swig/tesseract_srdf_python.i PACKAGE tesseract_srdf LIBS tesseract::srdf )
tesseract_python_module(tesseract_urdf_python SWIG_SRCS swig/tesseract_urdf_python.i PACKAGE tesseract_urdf LIBS tesseract::urdf )
tesseract_python_module(tesseract_state_solver_python SWIG_SRCS swig/tesseract_state_solver_python.i PACKAGE tesseract_state_solver LIBS tesseract::state_solver tesseract::state_solver_ofkt tesseract::state_solver_kdl)
tesseract_python_module(tesseract_kinematics_python SWIG_SRCS swig/tesseract_kinematics_python.i PACKAGE tesseract_kinematics LIBS tesseract::kinematics tesseract::tesseract_kinematics_kdl_factories tesseract::tesseract_kinematics_ur_factory tesseract::tesseract_kinematics_opw_factory tesseract::kinematics_factories )
tesseract_python_module(tesseract_environment_python SWIG_SRCS swig/tesseract_environment_python.i PACKAGE tesseract_environment LIBS tesseract::environment )
tesseract_python_module(tesseract_visualization_python SWIG_SRCS swig/tesseract_visualization_python.i PACKAGE tesseract_visualization LIBS tesseract::visualization)
tesseract_python_module(tesseract_command_language_python SWIG_SRCS swig/tesseract_command_language_python.i PACKAGE tesseract_command_language LIBS tesseract::command_language)
tesseract_python_module(tesseract_motion_planners_python SWIG_SRCS swig/tesseract_motion_planners_python.i PACKAGE tesseract_motion_planners LIBS tesseract::motion_planners_simple tesseract::motion_planners )
tesseract_python_module(tesseract_motion_planners_simple_python SWIG_SRCS swig/tesseract_motion_planners_simple_python.i PACKAGE tesseract_motion_planners_simple LIBS tesseract::motion_planners_simple )
tesseract_python_module(tesseract_motion_planners_trajopt_python SWIG_SRCS swig/tesseract_motion_planners_trajopt_python.i PACKAGE tesseract_motion_planners_trajopt LIBS tesseract::motion_planners_trajopt )
tesseract_python_module(tesseract_motion_planners_ompl_python SWIG_SRCS swig/tesseract_motion_planners_ompl_python.i PACKAGE tesseract_motion_planners_ompl LIBS tesseract::motion_planners_ompl )
tesseract_python_module(tesseract_motion_planners_descartes_python SWIG_SRCS swig/tesseract_motion_planners_descartes_python.i PACKAGE tesseract_motion_planners_descartes LIBS tesseract::motion_planners_descartes )
tesseract_python_module(tesseract_time_parameterization_python SWIG_SRCS swig/tesseract_time_parameterization_python.i PACKAGE tesseract_time_parameterization LIBS tesseract::time_parameterization tesseract::time_parameterization_isp tesseract::time_parameterization_totg tesseract::time_parameterization_ruckig)
tesseract_python_module(tesseract_task_composer_python SWIG_SRCS swig/tesseract_task_composer_python.i PACKAGE tesseract_task_composer LIBS tesseract::task_composer tesseract::task_composer_nodes tesseract::task_composer_taskflow tesseract::collision  tesseract::tesseract_task_composer_factories tesseract::tesseract_task_composer_planning_factories tesseract::tesseract_task_composer_planning_nodes tesseract::tesseract_task_composer_taskflow_factories tesseract::motion_planners_trajopt )

set(TESSERACT_PYTHON_MODULE_TARGETS 
  _tesseract_common_python
  _tesseract_geometry_python
  _tesseract_collision_python
  _tesseract_scene_graph_python
  _tesseract_srdf_python
  _tesseract_urdf_python
  _tesseract_state_solver_python
  _tesseract_kinematics_python
  _tesseract_environment_python
  _tesseract_visualization_python
  _tesseract_command_language_python
  _tesseract_motion_planners_python
  _tesseract_motion_planners_simple_python
  _tesseract_motion_planners_trajopt_python
  _tesseract_motion_planners_ompl_python
  _tesseract_motion_planners_descartes_python
  _tesseract_time_parameterization_python
  _tesseract_task_composer_python
)

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/setup.py.in" "${CMAKE_CURRENT_BINARY_DIR}/python/setup.py" @ONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/__package_init__.py.in ${CMAKE_CURRENT_BINARY_DIR}/python/tesseract_robotics/__init__.py @ONLY)

option(TESSERACT_PYTHON_BUILD_WHEEL "Build self contained Python wheel")
if(NOT TESSERACT_PYTHON_BUILD_WHEEL)

#install(CODE "MESSAGE(\"INSTALL TODO!\")")


install(CODE "
    message(STATUS \"Installing Python package to ${CMAKE_INSTALL_PREFIX}\")
    
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -m pip install .
            --root=$ENV{DESTDIR}/
            --prefix=${CMAKE_INSTALL_PREFIX}
            --no-deps
            --no-build-isolation
            --ignore-installed
        WORKING_DIRECTORY \"${CMAKE_CURRENT_BINARY_DIR}/python\"
        COMMAND_ERROR_IS_FATAL ANY
    )
")

  # Allows Colcon to find non-Ament packages when using workspace underlays
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv "prepend-non-duplicate;AMENT_PREFIX_PATH;")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv DESTINATION share/${PROJECT_NAME}/hook)


if (WIN32)
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv "prepend-non-duplicate;PYTHONPATH;lib/site-packages")
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/tesseract_python_dll_path.dsv "prepend-non-duplicate;TESSERACT_PYTHON_DLL_PATH;bin")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/tesseract_python_dll_path.dsv DESTINATION share/${PROJECT_NAME}/hook)    
else()
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv "prepend-non-duplicate;PYTHONPATH;lib/site-packages")
endif()
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv DESTINATION share/${PROJECT_NAME}/hook)

  
if(TESSERACT_ENABLE_TESTING)
  enable_testing()

  add_test (NAME python-tests
  COMMAND ${Python3_EXECUTABLE} -m pytest -p no:cacheprovider
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  
endif()

else()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/install_build_wheel.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/install_build_wheel.cmake @ONLY)

# install(SCRIPT ${CMAKE_CURRENT_BINARY_DIR}/install_build_wheel.cmake)
add_custom_target(tesseract_python_wheel ALL ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/install_build_wheel.cmake 
  DEPENDS ${TESSERACT_PYTHON_MODULE_TARGETS} WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endif()

if(TESSERACT_PYTHON_BUILD_DOCUMENTATION)
  find_program(SPHINX_EXECUTABLE NAMES sphinx-build)
    # Get absolute path of ${CMAKE_CURRENT_SOURCE_DIR}/../tesseract_viewer_python
    get_filename_component(ABS_TESSERACT_VIEWER_PYTHON ${CMAKE_SOURCE_DIR}/../tesseract_viewer_python ABSOLUTE)
    if(WIN32)
      set(docs_pythonpath "${CMAKE_BINARY_DIR}/python\;${ABS_TESSERACT_VIEWER_PYTHON}")
    else()
      set(docs_pythonpath "${CMAKE_BINARY_DIR}/python:${ABS_TESSERACT_VIEWER_PYTHON}")
    endif()

    if(NOT SPHINX_EXECUTABLE)
        message(FATAL_ERROR "Sphinx is required to build Python documentation")
    endif()
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/docs_build)
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/docs)
    add_custom_target(tesseract_python_doc
        COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${docs_pythonpath}
        ${SPHINX_EXECUTABLE} -E -a -b html
        ${CMAKE_SOURCE_DIR}/../docs
        ${CMAKE_BINARY_DIR}/docs
        COMMENT "Generating Python API documentation with Sphinx"
        DEPENDS ${TESSERACT_PYTHON_MODULE_TARGETS}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/docs_build
    )
endif()
