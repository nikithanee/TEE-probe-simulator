# Install script for directory: /uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/src/tee_probe

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/install/tee_probe")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tee_probe/" TYPE DIRECTORY FILES
    "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/src/tee_probe/launch"
    "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/src/tee_probe/config"
    "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/src/tee_probe/urdf"
    "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/src/tee_probe/scripts"
    "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/src/tee_probe/models"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tee_probe" TYPE PROGRAM FILES
    "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/src/tee_probe/scripts/drl_controller_node.py"
    "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/src/tee_probe/scripts/tee_probe_env.py"
    "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/src/tee_probe/scripts/train_tee_probe.py"
    "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/src/tee_probe/scripts/manual_control_node.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/build/tee_probe/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/tee_probe")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/build/tee_probe/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/tee_probe")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tee_probe/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tee_probe/environment" TYPE FILE FILES "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/build/tee_probe/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tee_probe/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tee_probe/environment" TYPE FILE FILES "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/build/tee_probe/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tee_probe" TYPE FILE FILES "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/build/tee_probe/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tee_probe" TYPE FILE FILES "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/build/tee_probe/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tee_probe" TYPE FILE FILES "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/build/tee_probe/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tee_probe" TYPE FILE FILES "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/build/tee_probe/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tee_probe" TYPE FILE FILES "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/build/tee_probe/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/build/tee_probe/ament_cmake_index/share/ament_index/resource_index/packages/tee_probe")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tee_probe/cmake" TYPE FILE FILES
    "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/build/tee_probe/ament_cmake_core/tee_probeConfig.cmake"
    "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/build/tee_probe/ament_cmake_core/tee_probeConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tee_probe" TYPE FILE FILES "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/src/tee_probe/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/uio/hume/student-u78/nikithan/tee_probe_ws/ros2_ws/build/tee_probe/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
