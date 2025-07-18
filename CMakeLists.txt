#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

cmake_minimum_required(VERSION 3.15)
if(POLICY CMP0063)
  cmake_policy(SET CMP0063 NEW)
endif()
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()

set(PROJECT_NAME mc_rtc)
set(PROJECT_DESCRIPTION mc_rtc)
set(PROJECT_VERSION 2.13.0)
set(PROJECT_URL https://github.com/jrl-umi3218/mc_rtc)
set(PROJECT_DEBUG_POSTFIX "_d")
set(PROJECT_USE_KEYWORD_LINK_LIBRARIES TRUE)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(PROJECT_CUSTOM_HEADER_DIR "mc_rtc/macros")
set(PROJECT_CUSTOM_HEADER_EXTENSION "h")
set(PROJECT_GENERATED_HEADERS_SKIP_CONFIG ON)
set(PROJECT_GENERATED_HEADERS_SKIP_WARNING ON)

set(DOXYGEN_USE_MATHJAX "YES")
set(DOXYGEN_DISTRIBUTE_GROUP_DOC "YES")
set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

include(cmake/base.cmake)
include(cmake/cython/cython.cmake)
include(cmake/msvc-specific.cmake)
include(cmake/version-script.cmake)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

project(${PROJECT_NAME} C CXX)
set(MC_RTC_SOURCE_DIR "${PROJECT_SOURCE_DIR}")

set_default_cmake_build_type(RelWithDebInfo)

if(NOT DEFINED MC_RTC_INSTALL_PREFIX)
  set(MC_RTC_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")
endif()

if(WIN32)
  set(MC_RTC_LOADER_DEBUG_SUFFIX "/debug")
else()
  set(MC_RTC_LOADER_DEBUG_SUFFIX "")
endif()

option(GENERATE_COVERAGE "Generate coverage data" FALSE)
if("${CMAKE_CXX_FLAGS}" MATCHES "--coverage")
  set(GENERATE_COVERAGE
      TRUE
      CACHE BOOL "Generate coverage data" FORCE
  )
  string(REPLACE "--coverage" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  string(REPLACE "--coverage" "" CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
  string(REPLACE "--coverage" "" CMAKE_MODULE_LINKER_FLAGS
                 "${CMAKE_MODULE_LINKER_FLAGS}"
  )
endif()

option(TREAT_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

option(BUILD_CONTROLLER_SAMPLES "Build sample controllers in the project" ON)
option(DISABLE_CONTROLLER_TESTS "Disable controller unit tests" OFF)
option(DISABLE_ROBOT_TESTS "Disable RobotModule unit tests" OFF)
option(ENABLE_FAST_TESTS "Run controllers tests for a few iterations" OFF)

option(BUILD_BENCHMARKS "Build benchmarks" OFF)

option(MC_RTC_BUILD_STATIC
       "Build a static version of mc_rtc, this has no support for pluggable components"
       OFF
)
option(MC_RTC_DISABLE_NETWORK "Build without network support" OFF)
option(MC_RTC_DEVELOPER_MODE
       "Disable exact version embedding to speed up recompilations" OFF
)

option(DISABLE_ROS "Build without ROS support (even if ROS was found)" OFF)

set(BOOST_STACKTRACE "")
set(BOOST_USE_STACKTRACE_DEFINE "")
function(find_best_boost_stacktrace)
  set(stacktrace_variants backtrace addr2line basic)
  if(WIN32)
    set(stacktrace_variants windbg ${stacktrace_variants})
  endif()
  foreach(variant ${stacktrace_variants})
    find_package(Boost QUIET COMPONENTS stacktrace_${variant})
    string(TOUPPER "${variant}" VARIANT)
    if(Boost_stacktrace_${variant}_FOUND OR Boost_STACKTRACE_${VARIANT}_FOUND)
      set(BOOST_STACKTRACE
          stacktrace_${variant}
          PARENT_SCOPE
      )
      set(BOOST_USE_STACKTRACE_DEFINE
          "BOOST_STACKTRACE_USE_${VARIANT}"
          PARENT_SCOPE
      )
      return()
    endif()
  endforeach()
endfunction()
find_best_boost_stacktrace()
if(BOOST_STACKTRACE STREQUAL "")
  message(
    FATAL_ERROR
      "No suitable Boost.Stacktrace installed, maybe you are missing some dependencies"
  )
endif()

add_project_dependency(Boost REQUIRED COMPONENTS filesystem timer ${BOOST_STACKTRACE})
add_project_dependency(Tasks REQUIRED)
add_project_dependency(TVM REQUIRED)
if(NOT TARGET RBDyn::Parsers)
  message(FATAL_ERROR "RBDyn must be built with parsers support")
endif()
add_project_dependency(eigen-quadprog REQUIRED)
add_project_dependency(state-observation REQUIRED)
add_project_dependency(ndcurves REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/CMakeModules)

if(NOT MC_RTC_BUILD_STATIC)
  # LTDL (defines target: mc_rtc_3rd_party::ltdl)
  add_subdirectory(3rd-party/ltdl)
endif()

# GEOS (defines/imports target: GEOS::geos)
find_package(mc_rtc_3rd_party_geos REQUIRED)

if(NOT MC_RTC_DISABLE_NETWORK)
  # nanomsg (defines/imports target: nanomsg)
  find_package(mc_rtc_3rd_party_nanomsg REQUIRED)
endif()

if(DEFINED ENV{WSL_DISTRO_NAME})
  set(HOST_IS_WSL ON)
else()
  set(HOST_IS_WSL OFF)
endif()

if(HOST_IS_WSL)
  set(WSL_NOTIFY_SEND_VERSION "v0.1.871612270")
  set(WSL_NOTIFY_SEND_SHA256
      8531436fd005844aca9948d68b9e4e40df56645a9eade91f47027cdd4d0a1b0e
  )
  file(
    DOWNLOAD
    https://github.com/stuartleeks/wsl-notify-send/releases/download/${WSL_NOTIFY_SEND_VERSION}/wsl-notify-send_windows_amd64.zip
    ${PROJECT_BINARY_DIR}/3rd-party/wsl-notify-send.zip
    EXPECTED_HASH SHA256=${WSL_NOTIFY_SEND_SHA256}
  )
  set(WSL_NOTIFY_SEND_DIR "${PROJECT_BINARY_DIR}/3rd-party/wsl-notify-send/")
  file(MAKE_DIRECTORY ${WSL_NOTIFY_SEND_DIR})
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E tar xf
            ${PROJECT_BINARY_DIR}/3rd-party/wsl-notify-send.zip
    WORKING_DIRECTORY "${WSL_NOTIFY_SEND_DIR}" COMMAND_ERROR_IS_FATAL ANY
  )
  set(WSL_NOTIFY_SEND_LOCATION "${WSL_NOTIFY_SEND_DIR}/wsl-notify-send.exe")
  message("-- Use wsl-notify-send for notifications at ${WSL_NOTIFY_SEND_LOCATION}")
endif()

if(NOT HOST_IS_WSL AND NOT WIN32)
  find_package(mc_rtc_3rd_party_libnotify REQUIRED)
  message("-- Use libnotify for notifications")
elseif(NOT HOST_IS_WSL AND WIN32)
  message("-- Use WinToast for notifications")
endif()

# qhull (build re-entrant static version)
add_subdirectory(3rd-party/qhull)

# mpack (build static version)
add_subdirectory(3rd-party/mpack)

# yaml-cpp
add_project_dependency(yaml-cpp 0.5.0 REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(YAML_CPP_TARGET yaml-cpp)
endif()

# RapidJSON The code was copied from the 1.1.0 release tarball for RapidJSON
# https://github.com/miloyip/rapidjson
add_library(mc_rtc_3rd_party::RapidJSON INTERFACE IMPORTED)
set_target_properties(
  mc_rtc_3rd_party::RapidJSON
  PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
             ${CMAKE_CURRENT_SOURCE_DIR}/3rd-party/RapidJSON
)

# spdlog
add_project_dependency(spdlog 1.5.0 REQUIRED)

macro(find_description_package PACKAGE)
  set(PACKAGE_PATH_VAR "${PACKAGE}_PATH")
  string(TOUPPER "${PACKAGE_PATH_VAR}" PACKAGE_PATH_VAR)
  find_package(${PACKAGE} REQUIRED)
  if("${${PACKAGE}_INSTALL_PREFIX}" STREQUAL "")
    if("${${PACKAGE}_SOURCE_PREFIX}" STREQUAL "")
       if("${${PACKAGE}_DIR}" STREQUAL "")
         message(
          FATAL_ERROR
            "Your ${PACKAGE} does not define where to find the data, please update."
         )
       else()
         set(${PACKAGE_PATH_VAR} "${${PACKAGE}_DIR}/..")
       endif()
    else()
      set(${PACKAGE_PATH_VAR} "${${PACKAGE}_SOURCE_PREFIX}")
    endif()
  else()
    set(${PACKAGE_PATH_VAR} "${${PACKAGE}_INSTALL_PREFIX}")
  endif()
  # Cleanup the path provided by CMake
  get_filename_component(${PACKAGE_PATH_VAR} "${${PACKAGE_PATH_VAR}}" REALPATH)
  message("-- Found ${PACKAGE}: ${${PACKAGE_PATH_VAR}}")
endmacro()

find_description_package(mc_env_description)
find_description_package(mc_int_obj_description)

macro(mc_rtc_set_prefix NAME FOLDER)
  set(MC_${NAME}_LIBRARY_INSTALL_PREFIX ${CMAKE_INSTALL_FULL_LIBDIR}/${FOLDER})
  if(WIN32)
    set(MC_${NAME}_RUNTIME_INSTALL_PREFIX ${CMAKE_INSTALL_FULL_BINDIR}/${FOLDER})
    set(MC_${NAME}_RUNTIME_DESTINATION_PREFIX
        ${MC_RTC_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/${FOLDER}
    )
  else()
    set(MC_${NAME}_RUNTIME_INSTALL_PREFIX ${CMAKE_INSTALL_FULL_LIBDIR}/${FOLDER})
    set(MC_${NAME}_RUNTIME_DESTINATION_PREFIX
        ${MC_RTC_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/${FOLDER}
    )
  endif()
  # For backward compatibility
  set(MC_${NAME}_INSTALL_PREFIX "${MC_${NAME}_LIBRARY_INSTALL_PREFIX}")
endmacro()

mc_rtc_set_prefix(PLUGINS mc_plugins)
mc_rtc_set_prefix(ROBOTS mc_robots)
mc_rtc_set_prefix(OBSERVERS mc_observers)
mc_rtc_set_prefix(CONTROLLER mc_controller)

set(MC_RTC_BINDIR "${CMAKE_INSTALL_FULL_BINDIR}")
set(MC_RTC_LIBDIR "${CMAKE_INSTALL_FULL_LIBDIR}")

configure_file(
  ${PROJECT_SOURCE_DIR}/include/mc_rtc/config.h.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/include/mc_rtc/config.h
)

# For MSVC, set local environment variable to enable finding the built dll of the main
# library when launching ctest with RUN_TESTS
if(MSVC)
  set(CMAKE_MSVCIDE_RUN_PATH
      "\$(SolutionDir)/3rd-party/ltdl/\$(Configuration);\$(SolutionDir)/src/\$(Configuration);\$(SolutionDir)/src/mc_control/fsm/\$(Configuration)"
  )
endif(MSVC)

include(CMakeModules/ListAllFiles.cmake)

add_subdirectory(CMakeModules)
add_subdirectory(doc)
add_subdirectory(src)
add_subdirectory(plugins)
add_subdirectory(utils)

if(${PYTHON_BINDING})
  add_subdirectory(binding/python)
endif()

if(${BUILD_TESTING})
  add_subdirectory(tests)
endif()
if(${BUILD_BENCHMARKS})
  add_subdirectory(benchmarks)
endif()

add_subdirectory(robots)
add_subdirectory(observers)
add_subdirectory(controllers)

install(
  DIRECTORY include ${CMAKE_BINARY_DIR}/include
  DESTINATION ${CMAKE_INSTALL_PREFIX}
  FILES_MATCHING
  REGEX "h(pp)?$"
)

file(GLOB data_files "data/*.json")
install(FILES ${data_files} DESTINATION share/mc_rtc)
install(FILES etc/mc_rtc.yaml DESTINATION etc)

# Extra macros for mc_rtc
file(RELATIVE_PATH mc_rtc_macros_RELATIVE_PATH
     "${CMAKE_INSTALL_PREFIX}/${CONFIG_INSTALL_DIR}" "${CMAKE_INSTALL_PREFIX}"
)
configure_file(
  src/mc_rtcMacros.in.cmake "${CMAKE_CURRENT_BINARY_DIR}/src/mc_rtcMacros.cmake" @ONLY
)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/src/mc_rtcMacros.cmake"
        DESTINATION "${CONFIG_INSTALL_DIR}"
)
set(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}
include(\"\${CMAKE_CURRENT_LIST_DIR}/mc_rtcMacros.cmake\")"
)

if(NOT MC_RTC_BUILD_STATIC)
  set(PACKAGE_EXTRA_MACROS
      "${PACKAGE_EXTRA_MACROS}
include(\"\${CMAKE_CURRENT_LIST_DIR}/mc_rtc_fsm_states.cmake\")
include(\"\${CMAKE_CURRENT_LIST_DIR}/mc_rtc_observers.cmake\")"
  )
endif()
