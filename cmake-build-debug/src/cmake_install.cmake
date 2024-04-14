# Install script for directory: /Users/chinzonghan/Documents/Berkeley SP24/CS 184/hw4-clothsim-sp24-coldplay/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/Users/chinzonghan/Documents/Berkeley SP24/CS 184/hw4-clothsim-sp24-coldplay/clothsim")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/Users/chinzonghan/Documents/Berkeley SP24/CS 184/hw4-clothsim-sp24-coldplay" TYPE EXECUTABLE FILES "/Users/chinzonghan/Documents/Berkeley SP24/CS 184/hw4-clothsim-sp24-coldplay/cmake-build-debug/clothsim")
  if(EXISTS "$ENV{DESTDIR}/Users/chinzonghan/Documents/Berkeley SP24/CS 184/hw4-clothsim-sp24-coldplay/clothsim" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/Users/chinzonghan/Documents/Berkeley SP24/CS 184/hw4-clothsim-sp24-coldplay/clothsim")
    execute_process(COMMAND /Users/chinzonghan/anaconda3/bin/install_name_tool
      -delete_rpath "/Users/chinzonghan/Documents/Berkeley SP24/CS 184/hw4-clothsim-sp24-coldplay/cmake-build-debug/ext/nanogui"
      "$ENV{DESTDIR}/Users/chinzonghan/Documents/Berkeley SP24/CS 184/hw4-clothsim-sp24-coldplay/clothsim")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/strip" -u -r "$ENV{DESTDIR}/Users/chinzonghan/Documents/Berkeley SP24/CS 184/hw4-clothsim-sp24-coldplay/clothsim")
    endif()
  endif()
endif()

