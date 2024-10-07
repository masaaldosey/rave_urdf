# configure build include directories
include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
                    ${TinyXML2_INCLUDE_DIRS}${urdf_INCLUDE_DIRS})

# link directories
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS}
                 ${TinyXML2_LIBRARY_DIRS}${urdf_LIBRARY_DIRS})

# create library
add_library(${PROJECT_NAME} SHARED raveurdf.cpp)

# set compiler and linker flags
set_target_properties(
  ${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}"
                             LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
# link libraries
target_link_libraries(${PROJECT_NAME} ${OpenRAVE_LIBRARIES})
# include directories
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)

# install library
install(TARGETS ${PROJECT_NAME} DESTINATION .)
