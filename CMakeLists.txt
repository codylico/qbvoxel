cmake_minimum_required(VERSION 3.0)
project(QBParse C)

if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  include(CTest)
endif (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

option(BUILD_TESTING "Build some tests" OFF)
option(QBParse_BUILD_TESTING "Build tests for QBParse" OFF)

set(qbparse_HEADERS
  include/qbparse/api.h)
set(qbparse_SOURCES
  src/qbparse/api.c)

add_library(qbparse ${qbparse_HEADERS} ${qbparse_SOURCES})
target_include_directories(qbparse PRIVATE include)

if (QBParse_BUILD_TESTING AND BUILD_TESTING)
  add_executable(qbparse_test tests/qbparse.c)
  target_link_libraries(qbparse_test qbparse)
  target_include_directories(qbparse_test PRIVATE include)
  add_test(NAME qbparse_test COMMAND qbparse_test)
endif (QBParse_BUILD_TESTING AND BUILD_TESTING)
