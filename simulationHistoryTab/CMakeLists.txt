# ***********************************
# CMakeLists.txt for SimulationHistoryTab
# 2013-01-15
# ***********************************
cmake_minimum_required (VERSION 2.6)
project (SimulationHistoryTab)

message (STATUS "Creating Makefile for SimulationHistoryTab")

IF(NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo Profile."
       FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

# Setting CMake flags
if(WIN32)
  set (CMAKE_INSTALL_PREFIX "C:/Golems" CACHE PATH "Install prefix" FORCE)
  message (STATUS "Setup Visual Studio Specific Flags")
  set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi /GL /Gy /W1 /EHsc")
  set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /Zi /GL /Gy /W1 /EHsc /arch:SSE2 /openmp")
  set (CMAKE_DEBUG_POSTFIX "d")
  set (CMAKE_EXE_LINKER_FLAGS "/LTCG")
  set (CMAKE_MODULE_LINKER_FLAGS "/LTCG")
elseif(APPLE)
  set(CMAKE_INSTALL_PREFIX /usr/local )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
else ()
  set(CMAKE_INSTALL_PREFIX /usr )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -std=c++0x")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/dart")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_INSTALL_PREFIX}/share/grip")

find_package(DARTExt REQUIRED)
find_package(DART REQUIRED CONFIG)
find_package(GRIP REQUIRED)

include_directories(${DARTExt_INCLUDE_DIRS})
include_directories(${DART_INCLUDE_DIRS})
include_directories(${GRIP_INCLUDE_DIRS})
link_directories(${DART_LIBRARY_DIRS})
link_directories(${GRIP_LIBRARY_DIRS})

# Search all header and source files
set ( SimulationHistoryTab_src SimulationHistoryTab.cpp SimulationHistoryTabApp.cpp )
set ( SimulationHistoryTab_hdr SimulationHistoryTab.h )

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
    add_executable (SimulationHistoryTab WIN32 ${SimulationHistoryTab_src} ${SimulationHistoryTab_hdr})
    set_target_properties ( SimulationHistoryTab PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
  else()
    add_executable (SimulationHistoryTab ${SimulationHistoryTab_src} ${SimulationHistoryTab_hdr})
  endif()

  # Link to GRIP, DART and wxWidgets
  target_link_libraries (SimulationHistoryTab ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${wxWidgets_LIBRARIES} )

else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)



