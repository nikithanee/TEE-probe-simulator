# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_tee_probe_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED tee_probe_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(tee_probe_FOUND FALSE)
  elseif(NOT tee_probe_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(tee_probe_FOUND FALSE)
  endif()
  return()
endif()
set(_tee_probe_CONFIG_INCLUDED TRUE)

# output package information
if(NOT tee_probe_FIND_QUIETLY)
  message(STATUS "Found tee_probe: 2.0.0 (${tee_probe_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'tee_probe' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${tee_probe_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(tee_probe_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${tee_probe_DIR}/${_extra}")
endforeach()
