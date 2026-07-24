# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_pladypos_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED pladypos_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(pladypos_FOUND FALSE)
  elseif(NOT pladypos_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(pladypos_FOUND FALSE)
  endif()
  return()
endif()
set(_pladypos_CONFIG_INCLUDED TRUE)

# output package information
if(NOT pladypos_FIND_QUIETLY)
  message(STATUS "Found pladypos: 1.0.0 (${pladypos_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'pladypos' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT pladypos_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(pladypos_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${pladypos_DIR}/${_extra}")
endforeach()
