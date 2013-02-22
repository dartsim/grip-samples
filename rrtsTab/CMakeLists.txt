# @file CMakeLists.xtxt
# @author Can Erdogan, Pushkar Kolhe
# @date Jan 30, 2013
# @brief The CMake file for the rrts project. The project demonstrates different RRT algorithms.
# See README.txt for more detail.

cmake_minimum_required (VERSION 2.6)
project (rrtsTab)

message (STATUS "Creating Makefile for rrtsTab")

# Setting CMake flags
if(WIN32)
  set (CMAKE_INSTALL_PREFIX "C:/Golems" CACHE PATH "Install prefix" FORCE)
  message (STATUS "Setup Visual Studio Specific Flags")
  set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi /GL /Gy /W1 /EHsc")
  set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /Zi /GL /Gy /W1 /EHsc")
  set (CMAKE_DEBUG_POSTFIX "d")
  set (CMAKE_EXE_LINKER_FLAGS "/LTCG")
  set (CMAKE_MODULE_LINKER_FLAGS "/LTCG")
else ()
  set (CMAKE_INSTALL_PREFIX /usr )
  # Optimization enabled
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -O3  -std=c++0x")
  set(CMAKE_CXX_FLAGS_PROFILE "-pg" CACHE STRING "Profiling flags")
endif()

# Include DART and GRIP config files
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/grip")

find_package(DARTExt REQUIRED)
find_package(DART REQUIRED)
find_package(GRIP REQUIRED)

include_directories(${DARTExt_INCLUDEDIR})
include_directories(${DART_INCLUDEDIR})
include_directories(${GRIP_INCLUDEDIR})
link_directories(${DART_LIBRARY_DIRS})
link_directories(${GRIP_LIBRARY_DIRS})

# Collect the source, script and test files
file(GLOB srcs "src/*.cpp")

# Include src in the file path
include_directories(src)

# Set wxWidgets-related parameters
if(WIN32)
  set(wxWidgets_EXCLUDE_COMMON_LIBRARIES TRUE)
  set(wxWidgets_USE_LIBS base core gl png tiff jpeg zlib)
else(WIN32)
  set(wxWidgets_USE_LIBS base core gl)
endif(WIN32)

# Include xWidgets
find_package (wxWidgets REQUIRED)
include (${wxWidgets_USE_FILE})
include_directories(${WXWIDGETS_SHARED_INCLUDE_DIRS})
link_directories(${WXWIDGETS_SHARED_LIBRARY_DIRS})

# Create the executable
if(MSVC)
	add_executable (rrtsTab WIN32 exe/rrtsApp.cpp ${srcs})
	set_target_properties (rrtsTab PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
else()
	add_executable (rrtsTab exe/rrtsApp.cpp ${srcs})
  add_custom_target(rrtsTab.run rrtsTab)
endif()

# Link to GRIP, DART and wxWidgets
target_link_libraries (rrtsTab ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} 
  ${wxWidgets_LIBRARIES} ${Boost_LIBRARIES})

# For Doxygen
find_package(Doxygen REQUIRED)
if(DOXYGEN_FOUND)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Generating API documentation with Doxygen" VERBATIM)
endif(DOXYGEN_FOUND)
