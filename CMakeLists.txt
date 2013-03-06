cmake_minimum_required(VERSION 2.8)
project(grip-samples)

set(GRIPS_MAJOR_VERSION "2")
set(GRIPS_MINOR_VERSION "4")
set(GRIPS_PATCH_VERSION "0precise")
set(PKG_DESC "GRIP Samples")
set(GRIPS_VERSION "${GRIPS_MAJOR_VERSION}.${GRIPS_MINOR_VERSION}.${GRIPS_PATCH_VERSION}")
set(PKG_EXTERNAL_DEPS "grip, dart")

# System Install paths
if(WIN32) 
  set(CMAKE_INSTALL_PREFIX "C:/Golems" CACHE PATH "Install prefix" FORCE)
elseif(APPLE)
  set(CMAKE_INSTALL_PREFIX "/usr/local" CACHE PATH "Install prefix" FORCE)
else()
  set(CMAKE_INSTALL_PREFIX "/usr" CACHE PATH "Install prefix" FORCE)
endif()

add_subdirectory(jacobianDemoTab)
add_subdirectory(planningTab)
add_subdirectory(simulationHistoryTab)
add_subdirectory(tabEmpty)
add_subdirectory(manipulationTab)

install(DIRECTORY ./ DESTINATION ${CMAKE_INSTALL_PREFIX}/share/grip/samples 
        FILES_MATCHING PATTERN "*.h"
                       PATTERN "*.cpp"
                       PATTERN "*.urdf"
                       PATTERN "*.dae"
                       PATTERN "*.3ds"
                       PATTERN "*.3DS"
                       PATTERN "*.STL"
                       PATTERN "*.stl"
                       PATTERN "CMakeLists.txt"
                       PATTERN "CMakeFiles*" EXCLUDE
                       PATTERN "*CPack*" EXCLUDE)
        
# Package Installer
set(CPACK_PACKAGE_NAME "grip-samples")
set(CPACK_SYSTEM_NAME "i386")
if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    set(CPACK_SYSTEM_NAME "amd64")    
endif()
set(CPACK_PACKAGE_VERSION_MAJOR ${GRIPS_MAJOR_VERSION})
set(CPACK_PACKAGE_VERSION_MINOR ${GRIPS_MINOR_VERSION})
set(CPACK_PACKAGE_VERSION_PATCH ${GRIPS_PATCH_VERSION})
set(CPACK_DEBIAN_PACKAGE_DEPENDS "grip (>=2.4.0), dart (>=2.4.0)")

set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}_${CPACK_SYSTEM_NAME}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Golems (http://github.com/golems/)")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PKG_DESC}")
set(CPACK_PACKAGE_VENDOR "Humanoids Robotics Lab at GT")

include(CPack)
