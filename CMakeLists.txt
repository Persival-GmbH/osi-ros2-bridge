cmake_policy(SET CMP0048 NEW)
cmake_minimum_required(VERSION 3.12)

project(osi-ros2-bridge)

####################################################################################################
# OSI Config
####################################################################################################
add_subdirectory(lib/open-simulation-interface)
get_directory_property(OSI_VERSION_MAJOR DIRECTORY lib/open-simulation-interface DEFINITION VERSION_MAJOR)
get_directory_property(OSI_VERSION_MINOR DIRECTORY lib/open-simulation-interface DEFINITION VERSION_MINOR)
get_directory_property(OSI_VERSION_PATCH DIRECTORY lib/open-simulation-interface DEFINITION VERSION_PATCH)
set(OSIVERSION "${OSI_VERSION_MAJOR}.${OSI_VERSION_MINOR}.${OSI_VERSION_PATCH}")
set(OSMPVERSION "1.3.0")

####################################################################################################
# FMI
####################################################################################################
include_directories(lib/fmi2/headers)
set(INSTALL_DIR "${CMAKE_BINARY_DIR}" CACHE PATH "Target directory for generated FMU")

add_subdirectory(src/)