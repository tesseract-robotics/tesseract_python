cmake_minimum_required(VERSION 3.5.0)

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

# find_package(tesseract_motion_planners REQUIRED)
# find_package(tesseract_common REQUIRED)
# find_package(tesseract_geometry REQUIRED)
# find_package(tesseract_visualization REQUIRED)
find_package(tesseract_collision REQUIRED)
# find_package(tesseract_urdf REQUIRED)
# find_package(tesseract_srdf REQUIRED)
find_package(tesseract_kinematics REQUIRED)
# find_package(tesseract_environment REQUIRED)
# find_package(tesseract_scene_graph REQUIRED)
# find_package(tesseract_command_language REQUIRED)
# find_package(tesseract_time_parameterization REQUIRED)
# find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)
# find_package(trajopt REQUIRED)
# find_package(TinyXML2 REQUIRED)
# find_package(opw_kinematics REQUIRED)

find_package(tesseract_task_composer REQUIRED COMPONENTS core planning taskflow)

if(NOT TinyXML2_INCLUDE_DIRS AND TARGET tinyxml2::tinyxml2)
get_target_property(TinyXML2_INCLUDE_DIRS tinyxml2::tinyxml2 INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS "Found from target interface: TinyXML2_INCLUDE_DIRS=${TinyXML2_INCLUDE_DIRS}")
endif()

tesseract_variables()

find_bullet()

include_directories(
  ${trajopt_INCLUDE_DIRS} 
  ${tesseract_common_INCLUDE_DIRS}
  ${tesseract_geometry_INCLUDE_DIRS}
  ${tesseract_collision_INCLUDE_DIRS}
  ${tesseract_motion_planners_INCLUDE_DIRS}
  ${tesseract_kinematics_INCLUDE_DIRS}
  ${tesseract_urdf_INCLUDE_DIRS}
  ${tesseract_scene_graph_INCLUDE_DIRS}
  ${tesseract_srdf_INCLUDE_DIRS}
  ${tesseract_environment_INCLUDE_DIRS}
  ${tesseract_command_language_INCLUDE_DIRS}
  ${tesseract_time_parameterization_INCLUDE_DIRS}
  ${tesseract_visualization_INCLUDE_DIRS}
  ${tesseract_task_composer_INCLUDE_DIRS}
  ${opw_kinematics_INCLUDE_DIRS}
  ${TinyXML2_INCLUDE_DIRS}
)

if(NOT PYTHON_VERSION)
  set(PYTHON_VERSION  $ENV{ROS_PYTHON_VERSION})
  message(STATUS "Setting Python Version: " ${PYTHON_VERSION})
endif()

if(PYTHON_VERSION)
  if(${PYTHON_VERSION} LESS 3)
    set(Python_ADDITIONAL_VERSIONS 2.7 2.6 2.5 2.4 2.3 2.2 2.1 2.0)
  else()
    set(Python_ADDITIONAL_VERSIONS 3.8 3.7 3.6 3.5 3.4 3.3 3.2 3.1 3.0)
  endif()
endif()

find_package(PythonInterp ${PYTHON_VERSION} REQUIRED)
find_package(PythonLibs REQUIRED)
message(STATUS "Building Python Version: " ${PYTHON_VERSION_MAJOR})

# Find NumPy
find_path(NUMPY_INCLUDE_NDARRAYOBJECT_DIR numpy/ndarrayobject.h PATHS ${PYTHON_INCLUDE_DIRS} NO_DEFAULT_PATH)
if(NUMPY_INCLUDE_NDARRAYOBJECT_DIR)
    set(NUMPY_INCLUDE_DIR ${NUMPY_INCLUDE_NDARRAYOBJECT_DIR})
else()
    execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import numpy; import os; print(os.path.dirname(numpy.__file__))"
                    RESULT_VARIABLE FIND_NUMPY_RESULT OUTPUT_VARIABLE NUMPY_INCLUDE_DIR1)
    string(STRIP "${NUMPY_INCLUDE_DIR1}" NUMPY_INCLUDE_DIR1)

    if(${FIND_NUMPY_RESULT})
        message(FATAL_ERROR "Could not determine NumPy include directory")
    endif()

    if(EXISTS "${NUMPY_INCLUDE_DIR1}/core/include/numpy/ndarrayobject.h")
        set(NUMPY_INCLUDE_DIR "${NUMPY_INCLUDE_DIR1}/core/include")
    else()
        if(EXISTS "${NUMPY_INCLUDE_DIR1}/_core/include/numpy/ndarrayobject.h")
            set(NUMPY_INCLUDE_DIR "${NUMPY_INCLUDE_DIR1}/_core/include")
        endif()
    endif()

endif()

if(NOT EXISTS "${NUMPY_INCLUDE_DIR}/numpy/ndarrayobject.h")
    message(FATAL_ERROR "Could not find numpy/ndarrayobject.h include file")
endif()

message(STATUS "NumPy Include Directory: ${NUMPY_INCLUDE_DIR}")

# End Find NumPy

if(${CMAKE_VERSION} VERSION_GREATER "3.13.0" OR ${CMAKE_VERSION} VERSION_EQUAL "3.13.0")
cmake_policy(SET CMP0078 OLD)
endif()

if(${CMAKE_VERSION} VERSION_GREATER "3.14.0" OR ${CMAKE_VERSION} VERSION_EQUAL "3.14.0")
cmake_policy(SET CMP0086 OLD)
endif()

include(FindSWIG)
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

tesseract_python_find_target(tesseract_collision_fcl_factories tesseract::tesseract_collision_core)
tesseract_python_find_target(tesseract_collision_bullet_factories tesseract::tesseract_collision_core)
tesseract_python_find_target(tesseract_kinematics_kdl_factories tesseract::tesseract_kinematics_core)
tesseract_python_find_target(tesseract_kinematics_ur_factory tesseract::tesseract_kinematics_core)
tesseract_python_find_target(tesseract_kinematics_opw_factory tesseract::tesseract_kinematics_core)
#tesseract_python_find_target(tesseract_kinematics_core_factories tesseract::tesseract_kinematics_core)
tesseract_python_find_target(tesseract_task_composer_factories tesseract::tesseract_task_composer)
tesseract_python_find_target(tesseract_task_composer_planning_factories tesseract::tesseract_task_composer)
# tesseract_python_find_target(tesseract_task_composer_planning_nodes tesseract::tesseract_task_composer)
tesseract_python_find_target(tesseract_task_composer_taskflow_factories tesseract::tesseract_task_composer)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/tesseract_python_module.cmake)

tesseract_python_module(tesseract_common_python SWIG_SRCS swig/tesseract_common_python.i PACKAGE tesseract_common LIBS tesseract::tesseract_common )
tesseract_python_module(tesseract_geometry_python SWIG_SRCS swig/tesseract_geometry_python.i PACKAGE tesseract_geometry LIBS tesseract::tesseract_geometry )
tesseract_python_module(tesseract_collision_python SWIG_SRCS swig/tesseract_collision_python.i PACKAGE tesseract_collision LIBS tesseract::tesseract_collision_core Bullet3::Bullet tesseract::tesseract_collision_bullet_factories tesseract::tesseract_collision_fcl_factories )
tesseract_python_module(tesseract_scene_graph_python SWIG_SRCS swig/tesseract_scene_graph_python.i PACKAGE tesseract_scene_graph LIBS tesseract::tesseract_scene_graph )
tesseract_python_module(tesseract_srdf_python SWIG_SRCS swig/tesseract_srdf_python.i PACKAGE tesseract_srdf LIBS tesseract::tesseract_srdf )
tesseract_python_module(tesseract_urdf_python SWIG_SRCS swig/tesseract_urdf_python.i PACKAGE tesseract_urdf LIBS tesseract::tesseract_urdf )
tesseract_python_module(tesseract_state_solver_python SWIG_SRCS swig/tesseract_state_solver_python.i PACKAGE tesseract_state_solver LIBS tesseract::tesseract_state_solver_core tesseract::tesseract_state_solver_ofkt tesseract::tesseract_state_solver_kdl)
tesseract_python_module(tesseract_kinematics_python SWIG_SRCS swig/tesseract_kinematics_python.i PACKAGE tesseract_kinematics LIBS tesseract::tesseract_kinematics_core tesseract::tesseract_kinematics_kdl_factories tesseract::tesseract_kinematics_ur_factory tesseract::tesseract_kinematics_opw_factory tesseract::tesseract_kinematics_core_factories )
tesseract_python_module(tesseract_environment_python SWIG_SRCS swig/tesseract_environment_python.i PACKAGE tesseract_environment LIBS tesseract::tesseract_environment )
tesseract_python_module(tesseract_visualization_python SWIG_SRCS swig/tesseract_visualization_python.i PACKAGE tesseract_visualization LIBS tesseract::tesseract_visualization)
tesseract_python_module(tesseract_command_language_python SWIG_SRCS swig/tesseract_command_language_python.i PACKAGE tesseract_command_language LIBS tesseract::tesseract_command_language)
tesseract_python_module(tesseract_motion_planners_python SWIG_SRCS swig/tesseract_motion_planners_python.i PACKAGE tesseract_motion_planners LIBS tesseract::tesseract_motion_planners_simple tesseract::tesseract_motion_planners_core )
tesseract_python_module(tesseract_motion_planners_simple_python SWIG_SRCS swig/tesseract_motion_planners_simple_python.i PACKAGE tesseract_motion_planners_simple LIBS tesseract::tesseract_motion_planners_simple )
tesseract_python_module(tesseract_motion_planners_trajopt_python SWIG_SRCS swig/tesseract_motion_planners_trajopt_python.i PACKAGE tesseract_motion_planners_trajopt LIBS tesseract::tesseract_motion_planners_trajopt )
tesseract_python_module(tesseract_motion_planners_ompl_python SWIG_SRCS swig/tesseract_motion_planners_ompl_python.i PACKAGE tesseract_motion_planners_ompl LIBS tesseract::tesseract_motion_planners_ompl )
tesseract_python_module(tesseract_motion_planners_descartes_python SWIG_SRCS swig/tesseract_motion_planners_descartes_python.i PACKAGE tesseract_motion_planners_descartes LIBS tesseract::tesseract_motion_planners_descartes )
tesseract_python_module(tesseract_time_parameterization_python SWIG_SRCS swig/tesseract_time_parameterization_python.i PACKAGE tesseract_time_parameterization LIBS tesseract::tesseract_time_parameterization_core tesseract::tesseract_time_parameterization_isp tesseract::tesseract_time_parameterization_totg tesseract::tesseract_time_parameterization_ruckig)
tesseract_python_module(tesseract_task_composer_python SWIG_SRCS swig/tesseract_task_composer_python.i PACKAGE tesseract_task_composer LIBS tesseract::tesseract_task_composer tesseract::tesseract_task_composer_nodes tesseract::tesseract_task_composer_taskflow tesseract::tesseract_collision_core  tesseract::tesseract_task_composer_factories tesseract::tesseract_task_composer_planning_factories tesseract::tesseract_task_composer_planning_nodes tesseract::tesseract_task_composer_taskflow_factories )

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


set(enable_setuptools_deb_layout OFF)
if(EXISTS "/etc/debian_version")
  set(enable_setuptools_deb_layout ON)
endif()
option(SETUPTOOLS_DEB_LAYOUT "Enable debian style python package layout" ${enable_setuptools_deb_layout})
if(SETUPTOOLS_DEB_LAYOUT)
  message(STATUS "Using Debian Python package layout")
  set(SETUPTOOLS_ARG_EXTRA "--install-layout=deb")
endif()

if (${PYTHON_VERSION_MAJOR} LESS 3)
    install(CODE "message(STATUS \"Running setup.py in ${CMAKE_CURRENT_BINARY_DIR}/python\")
    execute_process(COMMAND python2 setup.py install -f
--root=/ --prefix=${CMAKE_INSTALL_PREFIX} ${SETUPTOOLS_ARG_EXTRA} --single-version-externally-managed WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/python)")
else()
  install(CODE "message(STATUS \"Running setup.py in ${CMAKE_CURRENT_BINARY_DIR}/python\")
  execute_process(COMMAND ${PYTHON_EXECUTABLE} setup.py install -f
  --root=/ --prefix=${CMAKE_INSTALL_PREFIX} ${SETUPTOOLS_ARG_EXTRA} --single-version-externally-managed WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/python)")
endif()


  # Allows Colcon to find non-Ament packages when using workspace underlays
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv "prepend-non-duplicate;AMENT_PREFIX_PATH;")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv DESTINATION share/${PROJECT_NAME}/hook)

if (${PYTHON_VERSION_MAJOR} LESS 3)
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv "prepend-non-duplicate;PYTHONPATH;lib/python2.7/dist-packages")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv DESTINATION share/${PROJECT_NAME}/hook)
else()
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv "prepend-non-duplicate;PYTHONPATH;lib/python3/dist-packages")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv DESTINATION share/${PROJECT_NAME}/hook)
endif()
  
if(TESSERACT_ENABLE_TESTING)
  enable_testing()

  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})

  if (${TESSERACT_ENABLE_RUN_TESTING})
  add_dependencies(run_tests ${TESSERACT_PYTHON_MODULE_TARGETS})
    endif()

  add_test (NAME python-tests
  COMMAND ${PYTHON_EXECUTABLE} -m pytest -p no:cacheprovider
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  
  find_package(tesseract_support REQUIRED)
  if (WIN32)
  set_tests_properties(python-tests PROPERTIES ENVIRONMENT "TESSERACT_SUPPORT_DIR=${TESSERACT_SUPPORT_DIR}")
  else()
  set_tests_properties(python-tests PROPERTIES ENVIRONMENT "TESSERACT_SUPPORT_DIR=${TESSERACT_SUPPORT_DIR}")
  endif()
  

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
