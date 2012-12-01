cmake_minimum_required (VERSION 2.6)
project (rss_solvers CXX)

# The version number.
#set (BSS_VERSION_MAJOR 1)
#set (BSS_VERSION_MINOR 0)
#set (BSS_VERSION_REVISION 0)

# Adding the link command to libowl in this line.
# Should technically run a bunch of cmake scripts to discover where libowl
# was installed.
set(CMAKE_CXX_FLAGS "-O2 -std=c++0x -Wall -Wextra -Wno-sign-compare")
#set(CMAKE_CXX_FLAGS "-O3 -std=c++0x -Wall -Wextra -Wno-sign-compare")

FIND_PATH(OWL_INC_DIR NAMES owl PATHS /usr/include /usr/local/include)
IF(NOT OWL_INC_DIR)
  MESSAGE(FATAL_ERROR "Unable to find libowl installation!")
ELSE(NOT OWL_INC_DIR)
  #Truncate the search path
  STRING(REGEX REPLACE "/owl" "" OWL_INC_DIR ${OWL_INC_DIR})
ENDIF(NOT OWL_INC_DIR)

include_directories (include "${OWL_INC_DIR}")

add_subdirectory (src bin)

#Set the correct library directory suffix
if(NOT DEFINED LIB_SUFFIX)
  get_property(LIB64 GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS)
  if (${LIB64} STREQUAL "TRUE")
    set(LIBSUFFIX 64)
  else()
    set(LIBSUFFIX "")
  endif()
  set(LIB_SUFFIX ${LIBSUFFIX} CACHE STRING "lib directory suffix")
endif()

## the following are directories where stuff will be installed to
SET(PKGCONFIG_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}/pkgconfig/" CACHE STRING "Base directory for pkgconfig files")
SET(INCLUDE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/include/owl/" CACHE PATH "The subdirectory to the header prefix")

# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

