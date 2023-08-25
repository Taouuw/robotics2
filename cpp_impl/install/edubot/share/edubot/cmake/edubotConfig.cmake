# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_edubot_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED edubot_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(edubot_FOUND FALSE)
  elseif(NOT edubot_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(edubot_FOUND FALSE)
  endif()
  return()
endif()
set(_edubot_CONFIG_INCLUDED TRUE)

# output package information
if(NOT edubot_FIND_QUIETLY)
  message(STATUS "Found edubot: 0.0.0 (${edubot_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'edubot' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${edubot_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(edubot_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${edubot_DIR}/${_extra}")
endforeach()
