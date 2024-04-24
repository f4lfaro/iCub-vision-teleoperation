# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/fabiana/iCub/human-dynamics-estimation-vision2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/fabiana/iCub/human-dynamics-estimation-vision2/build

# Include any dependencies generated for this target.
include devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/compiler_depend.make

# Include the progress variables for this target.
include devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/progress.make

# Include the compile flags for this target's objects.
include devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/flags.make

devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/RobotPositionController.cpp.o: devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/flags.make
devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/RobotPositionController.cpp.o: /home/fabiana/iCub/human-dynamics-estimation-vision2/devices/RobotPositionController/RobotPositionController.cpp
devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/RobotPositionController.cpp.o: devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/fabiana/iCub/human-dynamics-estimation-vision2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/RobotPositionController.cpp.o"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/RobotPositionController && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/RobotPositionController.cpp.o -MF CMakeFiles/RobotPositionController.dir/RobotPositionController.cpp.o.d -o CMakeFiles/RobotPositionController.dir/RobotPositionController.cpp.o -c /home/fabiana/iCub/human-dynamics-estimation-vision2/devices/RobotPositionController/RobotPositionController.cpp

devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/RobotPositionController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/RobotPositionController.dir/RobotPositionController.cpp.i"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/RobotPositionController && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fabiana/iCub/human-dynamics-estimation-vision2/devices/RobotPositionController/RobotPositionController.cpp > CMakeFiles/RobotPositionController.dir/RobotPositionController.cpp.i

devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/RobotPositionController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/RobotPositionController.dir/RobotPositionController.cpp.s"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/RobotPositionController && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fabiana/iCub/human-dynamics-estimation-vision2/devices/RobotPositionController/RobotPositionController.cpp -o CMakeFiles/RobotPositionController.dir/RobotPositionController.cpp.s

devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/yarp_plugin_robot_position_controller.cpp.o: devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/flags.make
devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/yarp_plugin_robot_position_controller.cpp.o: devices/RobotPositionController/yarp_plugin_robot_position_controller.cpp
devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/yarp_plugin_robot_position_controller.cpp.o: devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/fabiana/iCub/human-dynamics-estimation-vision2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/yarp_plugin_robot_position_controller.cpp.o"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/RobotPositionController && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/yarp_plugin_robot_position_controller.cpp.o -MF CMakeFiles/RobotPositionController.dir/yarp_plugin_robot_position_controller.cpp.o.d -o CMakeFiles/RobotPositionController.dir/yarp_plugin_robot_position_controller.cpp.o -c /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/RobotPositionController/yarp_plugin_robot_position_controller.cpp

devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/yarp_plugin_robot_position_controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/RobotPositionController.dir/yarp_plugin_robot_position_controller.cpp.i"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/RobotPositionController && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/RobotPositionController/yarp_plugin_robot_position_controller.cpp > CMakeFiles/RobotPositionController.dir/yarp_plugin_robot_position_controller.cpp.i

devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/yarp_plugin_robot_position_controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/RobotPositionController.dir/yarp_plugin_robot_position_controller.cpp.s"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/RobotPositionController && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/RobotPositionController/yarp_plugin_robot_position_controller.cpp -o CMakeFiles/RobotPositionController.dir/yarp_plugin_robot_position_controller.cpp.s

# Object files for target RobotPositionController
RobotPositionController_OBJECTS = \
"CMakeFiles/RobotPositionController.dir/RobotPositionController.cpp.o" \
"CMakeFiles/RobotPositionController.dir/yarp_plugin_robot_position_controller.cpp.o"

# External object files for target RobotPositionController
RobotPositionController_EXTERNAL_OBJECTS =

lib/yarp/RobotPositionController.so: devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/RobotPositionController.cpp.o
lib/yarp/RobotPositionController.so: devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/yarp_plugin_robot_position_controller.cpp.o
lib/yarp/RobotPositionController.so: devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/build.make
lib/yarp/RobotPositionController.so: /usr/lib/lib/libidyntree-model.so
lib/yarp/RobotPositionController.so: /usr/lib/lib/libidyntree-core.so
lib/yarp/RobotPositionController.so: /home/fabiana/repos/robotology-superbuild/build/install/lib/libctrlLib.a
lib/yarp/RobotPositionController.so: /home/fabiana/repos/robotology-superbuild/build/install/lib/libYARP_dev.so.3.8.1
lib/yarp/RobotPositionController.so: /home/fabiana/repos/robotology-superbuild/build/install/lib/libYARP_init.so.3.8.1
lib/yarp/RobotPositionController.so: /home/fabiana/repos/robotology-superbuild/build/install/lib/libYARP_math.so.3.8.1
lib/yarp/RobotPositionController.so: /home/fabiana/repos/robotology-superbuild/build/install/lib/libYARP_sig.so.3.8.1
lib/yarp/RobotPositionController.so: /home/fabiana/repos/robotology-superbuild/build/install/lib/libYARP_os.so.3.8.1
lib/yarp/RobotPositionController.so: /usr/lib/x86_64-linux-gnu/libgsl.so
lib/yarp/RobotPositionController.so: /usr/lib/x86_64-linux-gnu/libgslcblas.so
lib/yarp/RobotPositionController.so: devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/fabiana/iCub/human-dynamics-estimation-vision2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared module ../../lib/yarp/RobotPositionController.so"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/RobotPositionController && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/RobotPositionController.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/build: lib/yarp/RobotPositionController.so
.PHONY : devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/build

devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/clean:
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/RobotPositionController && $(CMAKE_COMMAND) -P CMakeFiles/RobotPositionController.dir/cmake_clean.cmake
.PHONY : devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/clean

devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/depend:
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fabiana/iCub/human-dynamics-estimation-vision2 /home/fabiana/iCub/human-dynamics-estimation-vision2/devices/RobotPositionController /home/fabiana/iCub/human-dynamics-estimation-vision2/build /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/RobotPositionController /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : devices/RobotPositionController/CMakeFiles/RobotPositionController.dir/depend

