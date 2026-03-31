# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_crowd_statistics_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED crowd_statistics_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(crowd_statistics_FOUND FALSE)
  elseif(NOT crowd_statistics_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(crowd_statistics_FOUND FALSE)
  endif()
  return()
endif()
set(_crowd_statistics_CONFIG_INCLUDED TRUE)

# output package information
if(NOT crowd_statistics_FIND_QUIETLY)
  message(STATUS "Found crowd_statistics: 0.0.0 (${crowd_statistics_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'crowd_statistics' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${crowd_statistics_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(crowd_statistics_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${crowd_statistics_DIR}/${_extra}")
endforeach()
