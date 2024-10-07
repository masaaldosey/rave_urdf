cmake_minimum_required(VERSION 3.15)

# project name
project(raveurdf)

# configure C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# configure compiler options
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  add_definitions("-fno-strict-aliasing -Wall")
endif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)

# find dependencies
find_package(OpenRAVE 0.148 REQUIRED)
find_package(TinyXML2 REQUIRED)

# optional in case boost is used
find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)

# configure build
add_subdirectory(src)
