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
include remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/compiler_depend.make

# Include the progress variables for this target.
include remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/progress.make

# Include the compile flags for this target's objects.
include remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/flags.make

remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/HumanStateRemapper.cpp.o: remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/flags.make
remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/HumanStateRemapper.cpp.o: /home/fabiana/iCub/human-dynamics-estimation-vision2/remappers/HumanStateRemapper/HumanStateRemapper.cpp
remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/HumanStateRemapper.cpp.o: remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/fabiana/iCub/human-dynamics-estimation-vision2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/HumanStateRemapper.cpp.o"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/remappers/HumanStateRemapper && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/HumanStateRemapper.cpp.o -MF CMakeFiles/HumanStateRemapper.dir/HumanStateRemapper.cpp.o.d -o CMakeFiles/HumanStateRemapper.dir/HumanStateRemapper.cpp.o -c /home/fabiana/iCub/human-dynamics-estimation-vision2/remappers/HumanStateRemapper/HumanStateRemapper.cpp

remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/HumanStateRemapper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/HumanStateRemapper.dir/HumanStateRemapper.cpp.i"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/remappers/HumanStateRemapper && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fabiana/iCub/human-dynamics-estimation-vision2/remappers/HumanStateRemapper/HumanStateRemapper.cpp > CMakeFiles/HumanStateRemapper.dir/HumanStateRemapper.cpp.i

remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/HumanStateRemapper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/HumanStateRemapper.dir/HumanStateRemapper.cpp.s"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/remappers/HumanStateRemapper && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fabiana/iCub/human-dynamics-estimation-vision2/remappers/HumanStateRemapper/HumanStateRemapper.cpp -o CMakeFiles/HumanStateRemapper.dir/HumanStateRemapper.cpp.s

remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/yarp_plugin_human_state_remapper.cpp.o: remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/flags.make
remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/yarp_plugin_human_state_remapper.cpp.o: remappers/HumanStateRemapper/yarp_plugin_human_state_remapper.cpp
remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/yarp_plugin_human_state_remapper.cpp.o: remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/fabiana/iCub/human-dynamics-estimation-vision2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/yarp_plugin_human_state_remapper.cpp.o"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/remappers/HumanStateRemapper && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/yarp_plugin_human_state_remapper.cpp.o -MF CMakeFiles/HumanStateRemapper.dir/yarp_plugin_human_state_remapper.cpp.o.d -o CMakeFiles/HumanStateRemapper.dir/yarp_plugin_human_state_remapper.cpp.o -c /home/fabiana/iCub/human-dynamics-estimation-vision2/build/remappers/HumanStateRemapper/yarp_plugin_human_state_remapper.cpp

remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/yarp_plugin_human_state_remapper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/HumanStateRemapper.dir/yarp_plugin_human_state_remapper.cpp.i"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/remappers/HumanStateRemapper && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fabiana/iCub/human-dynamics-estimation-vision2/build/remappers/HumanStateRemapper/yarp_plugin_human_state_remapper.cpp > CMakeFiles/HumanStateRemapper.dir/yarp_plugin_human_state_remapper.cpp.i

remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/yarp_plugin_human_state_remapper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/HumanStateRemapper.dir/yarp_plugin_human_state_remapper.cpp.s"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/remappers/HumanStateRemapper && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fabiana/iCub/human-dynamics-estimation-vision2/build/remappers/HumanStateRemapper/yarp_plugin_human_state_remapper.cpp -o CMakeFiles/HumanStateRemapper.dir/yarp_plugin_human_state_remapper.cpp.s

# Object files for target HumanStateRemapper
HumanStateRemapper_OBJECTS = \
"CMakeFiles/HumanStateRemapper.dir/HumanStateRemapper.cpp.o" \
"CMakeFiles/HumanStateRemapper.dir/yarp_plugin_human_state_remapper.cpp.o"

# External object files for target HumanStateRemapper
HumanStateRemapper_EXTERNAL_OBJECTS =

lib/yarp/HumanStateRemapper.so: remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/HumanStateRemapper.cpp.o
lib/yarp/HumanStateRemapper.so: remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/yarp_plugin_human_state_remapper.cpp.o
lib/yarp/HumanStateRemapper.so: remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/build.make
lib/yarp/HumanStateRemapper.so: lib/libHumanStateMsg.so
lib/yarp/HumanStateRemapper.so: /home/fabiana/repos/robotology-superbuild/build/install/lib/libYARP_dev.so.3.8.1
lib/yarp/HumanStateRemapper.so: /home/fabiana/repos/robotology-superbuild/build/install/lib/libYARP_init.so.3.8.1
lib/yarp/HumanStateRemapper.so: /home/fabiana/repos/robotology-superbuild/build/install/lib/libYARP_math.so.3.8.1
lib/yarp/HumanStateRemapper.so: /home/fabiana/repos/robotology-superbuild/build/install/lib/libYARP_sig.so.3.8.1
lib/yarp/HumanStateRemapper.so: /home/fabiana/repos/robotology-superbuild/build/install/lib/libYARP_os.so.3.8.1
lib/yarp/HumanStateRemapper.so: remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/fabiana/iCub/human-dynamics-estimation-vision2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared module ../../lib/yarp/HumanStateRemapper.so"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/remappers/HumanStateRemapper && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/HumanStateRemapper.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/build: lib/yarp/HumanStateRemapper.so
.PHONY : remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/build

remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/clean:
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/remappers/HumanStateRemapper && $(CMAKE_COMMAND) -P CMakeFiles/HumanStateRemapper.dir/cmake_clean.cmake
.PHONY : remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/clean

remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/depend:
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fabiana/iCub/human-dynamics-estimation-vision2 /home/fabiana/iCub/human-dynamics-estimation-vision2/remappers/HumanStateRemapper /home/fabiana/iCub/human-dynamics-estimation-vision2/build /home/fabiana/iCub/human-dynamics-estimation-vision2/build/remappers/HumanStateRemapper /home/fabiana/iCub/human-dynamics-estimation-vision2/build/remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : remappers/HumanStateRemapper/CMakeFiles/HumanStateRemapper.dir/depend
