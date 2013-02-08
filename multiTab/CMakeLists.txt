cmake_minimum_required (VERSION 2.6)
project (multiTab)

message (STATUS "Creating Makefile for multiTab")

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
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -O3  -std=c++0x")
  set(CMAKE_CXX_FLAGS_PROFILE "-pg" CACHE STRING "Profiling flags")
endif()

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


# Search all header and source files
set ( planningTab_src "../planningTab/planningTab.cpp" "../planningTab/Controller.cpp" )
set ( planningTab_hdr "../planningTab/planningTab.h" "../planningTab/Controller.h" )
set ( VisualizationTab_src "../visualizationTab/VisualizationTab.cpp" )
set ( VisualizationTab_hdr "../visualizationTab/VisualizationTab.h" )
set ( MultiTab_src MultiTabApp.cpp )

if(WIN32)
  set (wxWidgets_EXCLUDE_COMMON_LIBRARIES TRUE)
  set (wxWidgets_USE_LIBS base core gl png tiff jpeg zlib)
else(WIN32)
  set (wxWidgets_USE_LIBS base core gl)
endif(WIN32)

find_package (wxWidgets)
if (wxWidgets_FOUND)
  include (${wxWidgets_USE_FILE})
  include_directories(${WXWIDGETS_SHARED_INCLUDE_DIRS})
  link_directories(${WXWIDGETS_SHARED_LIBRARY_DIRS})

  if(MSVC)
    add_executable (multiTab WIN32 ${MultiTab_src} ${planningTab_src} ${VisualizationTab_src} ${planningTab_hdr} ${VisualizationTab_hdr})
    set_target_properties (multiTab PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
  else()
    add_executable (multiTab ${planningTab_src} ${VisualizationTab_src} ${MultiTab_src} ${planningTab_hdr} ${VisualizationTab_hdr})
  endif()

  target_link_libraries (multiTab ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${wxWidgets_LIBRARIES}  ${Boost_LIBRARIES})

else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)



