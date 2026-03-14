# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/ws/build/fields2cover/_deps/steering_functions-src"
  "/home/ws/build/fields2cover/_deps/steering_functions-build"
  "/home/ws/build/fields2cover/_deps/steering_functions-subbuild/steering_functions-populate-prefix"
  "/home/ws/build/fields2cover/_deps/steering_functions-subbuild/steering_functions-populate-prefix/tmp"
  "/home/ws/build/fields2cover/_deps/steering_functions-subbuild/steering_functions-populate-prefix/src/steering_functions-populate-stamp"
  "/home/ws/build/fields2cover/_deps/steering_functions-subbuild/steering_functions-populate-prefix/src"
  "/home/ws/build/fields2cover/_deps/steering_functions-subbuild/steering_functions-populate-prefix/src/steering_functions-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/ws/build/fields2cover/_deps/steering_functions-subbuild/steering_functions-populate-prefix/src/steering_functions-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/ws/build/fields2cover/_deps/steering_functions-subbuild/steering_functions-populate-prefix/src/steering_functions-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
