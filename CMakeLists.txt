cmake_minimum_required( VERSION 2.8 )

# Create Project
project( 6.838_Final_Project )
 
# Find Packages
find_package( PCL 1.9 REQUIRED )
 
# include our src directory
include_directories(include)

# add all our source files
file(GLOB SOURCES "src/*.cpp")


if( PCL_FOUND )
  # Make sure PCL has proper dependencies
  include_directories( ${PCL_INCLUDE_DIRS} )
  link_directories( ${PCL_LIBRARY_DIRS} )
  add_definitions( ${PCL_DEFINITIONS} )

  add_executable( out ${SOURCES})

  # Additional Dependencies
  target_link_libraries( out ${PCL_LIBRARIES} )


endif()