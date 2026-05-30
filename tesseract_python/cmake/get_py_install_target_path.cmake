function(get_py_install_target_path out_target_path out_target_path_rel)
    
execute_process(
    COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/get_py_install_target_path.py" "${CMAKE_INSTALL_PREFIX}"
    OUTPUT_VARIABLE PYTHON_PURELIB_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    COMMAND_ERROR_IS_FATAL ANY
)

message(STATUS "Found Python install target ${PYTHON_PURELIB_DIR}")

set(${out_target_path} ${PYTHON_PURELIB_DIR} PARENT_SCOPE)

cmake_path(RELATIVE_PATH PYTHON_PURELIB_DIR BASE_DIRECTORY ${CMAKE_INSTALL_PREFIX} OUTPUT_VARIABLE PYTHON_PURELIB_DIR_REL)

if (out_target_path_rel)
    set(${out_target_path_rel} ${PYTHON_PURELIB_DIR_REL} PARENT_SCOPE)
endif()

endfunction()