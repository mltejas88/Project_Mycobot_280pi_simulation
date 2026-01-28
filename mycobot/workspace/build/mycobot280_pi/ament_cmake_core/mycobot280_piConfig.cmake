# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mycobot280_pi_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mycobot280_pi_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mycobot280_pi_FOUND FALSE)
  elseif(NOT mycobot280_pi_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mycobot280_pi_FOUND FALSE)
  endif()
  return()
endif()
set(_mycobot280_pi_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mycobot280_pi_FIND_QUIETLY)
  message(STATUS "Found mycobot280_pi: 0.1.0 (${mycobot280_pi_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mycobot280_pi' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT mycobot280_pi_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mycobot280_pi_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${mycobot280_pi_DIR}/${_extra}")
endforeach()
