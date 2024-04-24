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
include wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/compiler_depend.make

# Include the progress variables for this target.
include wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/progress.make

# Include the compile flags for this target's objects.
include wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/flags.make

wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/VisionTargetsWrapper.cpp.o: wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/flags.make
wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/VisionTargetsWrapper.cpp.o: /home/fabiana/iCub/human-dynamics-estimation-vision2/wrappers/VisionTargetsWrapper/VisionTargetsWrapper.cpp
wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/VisionTargetsWrapper.cpp.o: wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/fabiana/iCub/human-dynamics-estimation-vision2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/VisionTargetsWrapper.cpp.o"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/wrappers/VisionTargetsWrapper && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/VisionTargetsWrapper.cpp.o -MF CMakeFiles/VisionTargetsWrapper.dir/VisionTargetsWrapper.cpp.o.d -o CMakeFiles/VisionTargetsWrapper.dir/VisionTargetsWrapper.cpp.o -c /home/fabiana/iCub/human-dynamics-estimation-vision2/wrappers/VisionTargetsWrapper/VisionTargetsWrapper.cpp

wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/VisionTargetsWrapper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/VisionTargetsWrapper.dir/VisionTargetsWrapper.cpp.i"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/wrappers/VisionTargetsWrapper && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fabiana/iCub/human-dynamics-estimation-vision2/wrappers/VisionTargetsWrapper/VisionTargetsWrapper.cpp > CMakeFiles/VisionTargetsWrapper.dir/VisionTargetsWrapper.cpp.i

wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/VisionTargetsWrapper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/VisionTargetsWrapper.dir/VisionTargetsWrapper.cpp.s"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/wrappers/VisionTargetsWrapper && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fabiana/iCub/human-dynamics-estimation-vision2/wrappers/VisionTargetsWrapper/VisionTargetsWrapper.cpp -o CMakeFiles/VisionTargetsWrapper.dir/VisionTargetsWrapper.cpp.s

wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/yarp_plugin_vision_targets_wrapper.cpp.o: wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/flags.make
wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/yarp_plugin_vision_targets_wrapper.cpp.o: wrappers/VisionTargetsWrapper/yarp_plugin_vision_targets_wrapper.cpp
wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/yarp_plugin_vision_targets_wrapper.cpp.o: wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/fabiana/iCub/human-dynamics-estimation-vision2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/yarp_plugin_vision_targets_wrapper.cpp.o"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/wrappers/VisionTargetsWrapper && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/yarp_plugin_vision_targets_wrapper.cpp.o -MF CMakeFiles/VisionTargetsWrapper.dir/yarp_plugin_vision_targets_wrapper.cpp.o.d -o CMakeFiles/VisionTargetsWrapper.dir/yarp_plugin_vision_targets_wrapper.cpp.o -c /home/fabiana/iCub/human-dynamics-estimation-vision2/build/wrappers/VisionTargetsWrapper/yarp_plugin_vision_targets_wrapper.cpp

wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/yarp_plugin_vision_targets_wrapper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/VisionTargetsWrapper.dir/yarp_plugin_vision_targets_wrapper.cpp.i"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/wrappers/VisionTargetsWrapper && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fabiana/iCub/human-dynamics-estimation-vision2/build/wrappers/VisionTargetsWrapper/yarp_plugin_vision_targets_wrapper.cpp > CMakeFiles/VisionTargetsWrapper.dir/yarp_plugin_vision_targets_wrapper.cpp.i

wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/yarp_plugin_vision_targets_wrapper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/VisionTargetsWrapper.dir/yarp_plugin_vision_targets_wrapper.cpp.s"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/wrappers/VisionTargetsWrapper && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fabiana/iCub/human-dynamics-estimation-vision2/build/wrappers/VisionTargetsWrapper/yarp_plugin_vision_targets_wrapper.cpp -o CMakeFiles/VisionTargetsWrapper.dir/yarp_plugin_vision_targets_wrapper.cpp.s

# Object files for target VisionTargetsWrapper
VisionTargetsWrapper_OBJECTS = \
"CMakeFiles/VisionTargetsWrapper.dir/VisionTargetsWrapper.cpp.o" \
"CMakeFiles/VisionTargetsWrapper.dir/yarp_plugin_vision_targets_wrapper.cpp.o"

# External object files for target VisionTargetsWrapper
VisionTargetsWrapper_EXTERNAL_OBJECTS =

lib/yarp/VisionTargetsWrapper.so: wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/VisionTargetsWrapper.cpp.o
lib/yarp/VisionTargetsWrapper.so: wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/yarp_plugin_vision_targets_wrapper.cpp.o
lib/yarp/VisionTargetsWrapper.so: wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/build.make
lib/yarp/VisionTargetsWrapper.so: lib/libVisionTargetsMsg.so
lib/yarp/VisionTargetsWrapper.so: /home/fabiana/repos/robotology-superbuild/build/install/lib/libYARP_dev.so.3.8.1
lib/yarp/VisionTargetsWrapper.so: /home/fabiana/repos/robotology-superbuild/build/install/lib/libYARP_init.so.3.8.1
lib/yarp/VisionTargetsWrapper.so: /usr/lib/lib/libidyntree-core.so
lib/yarp/VisionTargetsWrapper.so: /home/fabiana/repos/robotology-superbuild/build/install/lib/libYARP_math.so.3.8.1
lib/yarp/VisionTargetsWrapper.so: /home/fabiana/repos/robotology-superbuild/build/install/lib/libYARP_sig.so.3.8.1
lib/yarp/VisionTargetsWrapper.so: /home/fabiana/repos/robotology-superbuild/build/install/lib/libYARP_os.so.3.8.1
lib/yarp/VisionTargetsWrapper.so: wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/fabiana/iCub/human-dynamics-estimation-vision2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared module ../../lib/yarp/VisionTargetsWrapper.so"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/wrappers/VisionTargetsWrapper && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/VisionTargetsWrapper.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/build: lib/yarp/VisionTargetsWrapper.so
.PHONY : wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/build

wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/clean:
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/wrappers/VisionTargetsWrapper && $(CMAKE_COMMAND) -P CMakeFiles/VisionTargetsWrapper.dir/cmake_clean.cmake
.PHONY : wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/clean

wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/depend:
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fabiana/iCub/human-dynamics-estimation-vision2 /home/fabiana/iCub/human-dynamics-estimation-vision2/wrappers/VisionTargetsWrapper /home/fabiana/iCub/human-dynamics-estimation-vision2/build /home/fabiana/iCub/human-dynamics-estimation-vision2/build/wrappers/VisionTargetsWrapper /home/fabiana/iCub/human-dynamics-estimation-vision2/build/wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : wrappers/VisionTargetsWrapper/CMakeFiles/VisionTargetsWrapper.dir/depend
