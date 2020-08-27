cmake_minimum_required(VERSION 3.0)

project(QBParse C)

set(qbparse_HEADERS
  include/qbparse/api.h)
set(qbparse_SOURCES
  src/qbparse/api.c)

add_library(qbparse ${qbparse_HEADERS} ${qbparse_SOURCES})
target_include_directories(qbparse PRIVATE include)
