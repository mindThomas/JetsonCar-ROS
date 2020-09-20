# Configure build as release (for maximum optimization) but still add debug information (-g flag)
set(CMAKE_BUILD_TYPE "Release")

#message(STATUS "Adding library: utils [${CMAKE_BUILD_TYPE}]")
message(STATUS "   " ${CMAKE_CURRENT_SOURCE_DIR} "\t\t[${CMAKE_BUILD_TYPE}]")

# Make a compiler flag for the current source folder path
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__SOURCE_FOLDER__='\"${CMAKE_CURRENT_SOURCE_DIR}\"'")

set(library_dependencies   
  curl
)

FILE(GLOB_RECURSE binary_sources *.cpp) # find all cpp files
FILE(GLOB_RECURSE binary_headers *.hpp) # find all hpp files
FILE(GLOB_RECURSE general_headers *.h) # find all hpp files

set(GLOBAL_HEADERS ${GLOBAL_HEADERS} ${binary_headers} ${general_headers} PARENT_SCOPE)

add_library( utils SHARED ${binary_sources} ${GLOBAL_HEADERS} )
target_link_libraries( utils ${library_dependencies} )
target_include_directories( utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} )

set_target_properties( utils PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -Wextra" ) # enable debugging
