# ***********************************
# CMakeLists.txt for cubesTab
# 2013-01-15
# ***********************************
cmake_minimum_required (VERSION 2.6)
project (cubesTab)

message (STATUS "Creating Makefile for cubesTab")

# Setting CMake flags
if(WIN32)
  message(STATUS "Setup Visual Studio Specific Flags")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi /Gy /W1 /EHsc")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /Zi /GL /Gy /W1 /EHsc")
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/LTCG")
elseif(CMAKE_COMPILER_IS_GNUCXX OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set (CMAKE_INSTALL_PREFIX /usr)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
  set(CMAKE_CXX_FLAGS_PROFILE "-pg" CACHE STRING "Profiling flags") 
  option(ENABLE_MSSE2 "Enable -msse2 flag" ON)
else()
  message(SEND_ERROR "Compiler not supported.")
endif()

if(APPLE)
  set (CMAKE_INSTALL_PREFIX /usr/local)
endif()

set(CMAKE_DEBUG_POSTFIX "d")

# Include DART and GRIP config files
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/grip")

find_package(DARTExt REQUIRED)
find_package(DART REQUIRED CONFIG)
find_package(GRIP REQUIRED)

include_directories(${DARTExt_INCLUDEDIR})
include_directories(${DART_INCLUDEDIR})
include_directories(${GRIP_INCLUDEDIR})
link_directories(${DART_LIBRARY_DIRS})
link_directories(${GRIP_LIBRARY_DIRS})

# Search all header and source files
set ( cubesTab_src cubesTab.cpp cubesTabApp.cpp )
set ( cubesTab_hdr cubesTab.h )

# Set wxWidgets-related parameters
if(WIN32)
  set (wxWidgets_EXCLUDE_COMMON_LIBRARIES TRUE)
  set (wxWidgets_USE_LIBS base core gl png tiff jpeg zlib)
else(WIN32)
  set (wxWidgets_USE_LIBS base core gl)
endif(WIN32)

# Search wxWidgets
find_package (wxWidgets)
if (wxWidgets_FOUND)
  include (${wxWidgets_USE_FILE})
  include_directories(${WXWIDGETS_SHARED_INCLUDE_DIRS})
  link_directories(${WXWIDGETS_SHARED_LIBRARY_DIRS})

  # If wxWidgets found, create the executable
  if(MSVC)
    add_executable (cubesTab WIN32 ${cubesTab_src} ${cubesTab_hdr})
    set_target_properties ( cubesTab PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
  else()
    add_executable (cubesTab ${cubesTab_src} ${cubesTab_hdr})
  endif()

  # Link to GRIP, DART and wxWidgets
  target_link_libraries (cubesTab ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${wxWidgets_LIBRARIES} ${Boost_LIBRARIES})

else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)



