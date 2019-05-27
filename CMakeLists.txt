######################
# set up the project #
######################
cmake_minimum_required(VERSION 2.8.3)

project(mpi_cpp_tools)

# required to use std::shared_ptr in code
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")

# ensuring path to libraries are set during install
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

############################
# Search for dependencies. #
############################

set(CATKIN_PKGS ${CATKIN_PKGS}
  mpi_cmake_modules
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})

search_for_eigen()

######################################################
# define the include directory of all ${CATKIN_PKGS} #
######################################################
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

########################################################
# manage the creation of the libraries and executables #
########################################################
#add_library(${lib_name} SHARED ${lib_name}.cpp)
#target_link_libraries(${lib_name} ${catkin_LIBRARIES})
#set(LIBRARIES_TO_BE_EXPORTED ${LIBRARIES_TO_BE_EXPORTED} ${lib_name})

##########################################
# export the package as a catkin package #
##########################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES_TO_BE_EXPORTED}
  CATKIN_DEPENDS ${CATKIN_PKGS}
)

#########################
# manage the unit tests #
#########################
catkin_add_gtest(test_linear_dynamics
  tests/main.cpp
  tests/test_linear_dynamics.cpp
)
target_link_libraries(test_linear_dynamics
  ${catkin_LIBRARIES}
)

##########################
# building documentation #
##########################
build_doxygen_documentation()