cmake_minimum_required (VERSION 3.15)

# project name
project(raveurdf)

# configure C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# configure compiler options
if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# find dependencies
find_package(OpenRAVE 0.148 REQUIRED)

# optional in case boost is used
find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)

## configure build
# include directories
include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
# link directories
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

# create library
add_library(raveurdf SHARED raveurdf.cpp)

# set compiler and linker flags
set_target_properties(raveurdf PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
# link libraries
target_link_libraries(raveurdf ${OpenRAVE_LIBRARIES})

# install library
install(TARGETS raveurdf DESTINATION .)
