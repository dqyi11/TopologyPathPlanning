cmake_minimum_required(VERSION 2.8)
project(topologyPathPlanning)

set(HARRTS_VERSION 1.0)

set(INCLUDE_INSTALL_DIR "include")
set(LIBRARY_INSTALL_DIR "lib")
set(CONFIG_INSTALL_DIR "${LIBRARY_INSTALL_DIR}/${PRJECT_NAME}/cmake")

include_directories("include")

add_subdirectory(src)

#==========================================================================
# Installation.
# 
install(DIRECTORY "include/"
  DESTINATION "${INCLUDE_INSTALL_DIR}")

install(FILES "package.xml"
  DESTINATION "share/${PROJECT_NAME}")
