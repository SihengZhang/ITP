# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/sihengzhang/Documents/USC gradute classes/CSCI599-30015D/final/final-project-flip-edges/cmake-build-debug/_deps/glfw-src"
  "/Users/sihengzhang/Documents/USC gradute classes/CSCI599-30015D/final/final-project-flip-edges/cmake-build-debug/_deps/glfw-build"
  "/Users/sihengzhang/Documents/USC gradute classes/CSCI599-30015D/final/final-project-flip-edges/cmake-build-debug/_deps/glfw-subbuild/glfw-populate-prefix"
  "/Users/sihengzhang/Documents/USC gradute classes/CSCI599-30015D/final/final-project-flip-edges/cmake-build-debug/_deps/glfw-subbuild/glfw-populate-prefix/tmp"
  "/Users/sihengzhang/Documents/USC gradute classes/CSCI599-30015D/final/final-project-flip-edges/cmake-build-debug/_deps/glfw-subbuild/glfw-populate-prefix/src/glfw-populate-stamp"
  "/Users/sihengzhang/Documents/USC gradute classes/CSCI599-30015D/final/final-project-flip-edges/cmake-build-debug/_deps/glfw-subbuild/glfw-populate-prefix/src"
  "/Users/sihengzhang/Documents/USC gradute classes/CSCI599-30015D/final/final-project-flip-edges/cmake-build-debug/_deps/glfw-subbuild/glfw-populate-prefix/src/glfw-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/sihengzhang/Documents/USC gradute classes/CSCI599-30015D/final/final-project-flip-edges/cmake-build-debug/_deps/glfw-subbuild/glfw-populate-prefix/src/glfw-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/sihengzhang/Documents/USC gradute classes/CSCI599-30015D/final/final-project-flip-edges/cmake-build-debug/_deps/glfw-subbuild/glfw-populate-prefix/src/glfw-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
