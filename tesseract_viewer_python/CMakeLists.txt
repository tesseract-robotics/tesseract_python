cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version})

find_package(Python3 COMPONENTS Interpreter REQUIRED)

# Read version
FILE (STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" tesseract_viewer_python_version1 REGEX "<version>[0-9]+\\.[0-9]+\\.[0-9]+</version>")
IF ("${tesseract_viewer_python_version1}" STREQUAL "" )
MESSAGE(FATAL_ERROR "Could not read tesseract_viewer_python version.")
ENDIF()
STRING (REGEX REPLACE "<version>([0-9]+\\.[0-9]+\\.[0-9]+)</version>" "\\1" tesseract_viewer_python_version2 ${tesseract_viewer_python_version1})
STRING (STRIP ${tesseract_viewer_python_version2} tesseract_viewer_python_version)
MESSAGE(STATUS "tesseract_viewer_python version: ${tesseract_viewer_python_version}")

file(TO_CMAKE_PATH "${Python3_EXECUTABLE}" Python3_EXECUTABLE_NORMED)

include(../tesseract_python/cmake/get_py_install_target_path.cmake)

get_py_install_target_path(PY_INSTALL_TARGET_PATH PY_INSTALL_TARGET_PATH_REL)

if(NOT TESSERACT_PYTHON_BUILD_WHEEL)

install(CODE "
    message(STATUS \"Installing Python package to ${PY_INSTALL_TARGET_PATH}\")
    
    execute_process(
        COMMAND ${Python3_EXECUTABLE_NORMED} -m pip install ${CMAKE_CURRENT_SOURCE_DIR}
            --root=$ENV{DESTDIR}/
            --target=${PY_INSTALL_TARGET_PATH}
            --no-deps
            --no-build-isolation
            --ignore-installed
        WORKING_DIRECTORY \"${CMAKE_CURRENT_SOURCE_DIR}\"
        COMMAND_ERROR_IS_FATAL ANY
    )
")

  # Allows Colcon to find non-Ament packages when using workspace underlays
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv "prepend-non-duplicate;AMENT_PREFIX_PATH;")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv DESTINATION share/${PROJECT_NAME}/hook)

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv "prepend-non-duplicate;PYTHONPATH;${PY_INSTALL_TARGET_PATH_REL}")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv DESTINATION share/${PROJECT_NAME}/hook)

else()
add_custom_target(tesseract_viewer_python_wheel ALL
    COMMAND ${Python3_EXECUTABLE} -m build --wheel --no-isolation
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    VERBATIM
)
endif()
