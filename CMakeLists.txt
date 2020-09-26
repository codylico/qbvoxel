# SPDX-License-Identifier: Unlicense
cmake_minimum_required(VERSION 3.0)
project(QBVoxel C)

if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  include(CTest)
endif (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

option(BUILD_TESTING "Build some tests" OFF)
option(QBVoxel_BUILD_TESTING "Build tests for QBVoxel" OFF)
option(QBVoxel_BUILD_TOOLS "Build tools for QBVoxel" OFF)

set(qbvoxel_HEADERS
  include/qbvoxel/api.h
  include/qbvoxel/parse.h
  include/qbvoxel/gen.h)
set(qbvoxel_SOURCES
  src/qbvoxel/api.c
  src/qbvoxel/parse.c
  src/qbvoxel/gen.c)

add_library(qbvoxel ${qbvoxel_HEADERS} ${qbvoxel_SOURCES})
target_include_directories(qbvoxel PRIVATE include)

if (WIN32 AND BUILD_SHARED_LIBS)
  target_compile_definitions(qbvoxel
    PUBLIC "QBVoxel_WIN32_DLL")
endif(WIN32 AND BUILD_SHARED_LIBS)

#based on https://cliutils.gitlab.io/modern-cmake/chapters/testing.html
if (QBVoxel_BUILD_TESTING AND BUILD_TESTING)
  add_executable(qbvoxel_test tests/qbvoxel.c)
  target_link_libraries(qbvoxel_test qbvoxel)
  target_include_directories(qbvoxel_test PRIVATE include)
  add_test(NAME qbvoxel_test COMMAND qbvoxel_test)
endif (QBVoxel_BUILD_TESTING AND BUILD_TESTING)
if (QBVoxel_BUILD_TOOLS)
  add_executable(qbvoxel_churn tools/churn.c)
  target_link_libraries(qbvoxel_churn qbvoxel)
  target_include_directories(qbvoxel_churn PRIVATE include)

  add_executable(qbvoxel_render tools/render.c)
  target_link_libraries(qbvoxel_render qbvoxel)
  target_include_directories(qbvoxel_render PRIVATE include)
  if (UNIX)
    find_library(m_LIBRARY NAMES m)
    target_link_libraries(qbvoxel_render "${m_LIBRARY}")
  endif (UNIX)
endif (QBVoxel_BUILD_TOOLS)
