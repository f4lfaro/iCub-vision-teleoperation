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
include devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/compiler_depend.make

# Include the progress variables for this target.
include devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/progress.make

# Include the compile flags for this target's objects.
include devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/flags.make

devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/HumanDynamicsEstimator.cpp.o: devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/flags.make
devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/HumanDynamicsEstimator.cpp.o: /home/fabiana/iCub/human-dynamics-estimation-vision2/devices/HumanDynamicsEstimator/HumanDynamicsEstimator.cpp
devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/HumanDynamicsEstimator.cpp.o: devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/fabiana/iCub/human-dynamics-estimation-vision2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/HumanDynamicsEstimator.cpp.o"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/HumanDynamicsEstimator && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/HumanDynamicsEstimator.cpp.o -MF CMakeFiles/HumanDynamicsEstimator.dir/HumanDynamicsEstimator.cpp.o.d -o CMakeFiles/HumanDynamicsEstimator.dir/HumanDynamicsEstimator.cpp.o -c /home/fabiana/iCub/human-dynamics-estimation-vision2/devices/HumanDynamicsEstimator/HumanDynamicsEstimator.cpp

devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/HumanDynamicsEstimator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/HumanDynamicsEstimator.dir/HumanDynamicsEstimator.cpp.i"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/HumanDynamicsEstimator && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fabiana/iCub/human-dynamics-estimation-vision2/devices/HumanDynamicsEstimator/HumanDynamicsEstimator.cpp > CMakeFiles/HumanDynamicsEstimator.dir/HumanDynamicsEstimator.cpp.i

devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/HumanDynamicsEstimator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/HumanDynamicsEstimator.dir/HumanDynamicsEstimator.cpp.s"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/HumanDynamicsEstimator && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fabiana/iCub/human-dynamics-estimation-vision2/devices/HumanDynamicsEstimator/HumanDynamicsEstimator.cpp -o CMakeFiles/HumanDynamicsEstimator.dir/HumanDynamicsEstimator.cpp.s

devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/yarp_plugin_human_dynamics_estimator.cpp.o: devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/flags.make
devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/yarp_plugin_human_dynamics_estimator.cpp.o: devices/HumanDynamicsEstimator/yarp_plugin_human_dynamics_estimator.cpp
devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/yarp_plugin_human_dynamics_estimator.cpp.o: devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/fabiana/iCub/human-dynamics-estimation-vision2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/yarp_plugin_human_dynamics_estimator.cpp.o"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/HumanDynamicsEstimator && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/yarp_plugin_human_dynamics_estimator.cpp.o -MF CMakeFiles/HumanDynamicsEstimator.dir/yarp_plugin_human_dynamics_estimator.cpp.o.d -o CMakeFiles/HumanDynamicsEstimator.dir/yarp_plugin_human_dynamics_estimator.cpp.o -c /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/HumanDynamicsEstimator/yarp_plugin_human_dynamics_estimator.cpp

devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/yarp_plugin_human_dynamics_estimator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/HumanDynamicsEstimator.dir/yarp_plugin_human_dynamics_estimator.cpp.i"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/HumanDynamicsEstimator && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/HumanDynamicsEstimator/yarp_plugin_human_dynamics_estimator.cpp > CMakeFiles/HumanDynamicsEstimator.dir/yarp_plugin_human_dynamics_estimator.cpp.i

devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/yarp_plugin_human_dynamics_estimator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/HumanDynamicsEstimator.dir/yarp_plugin_human_dynamics_estimator.cpp.s"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/HumanDynamicsEstimator && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/HumanDynamicsEstimator/yarp_plugin_human_dynamics_estimator.cpp -o CMakeFiles/HumanDynamicsEstimator.dir/yarp_plugin_human_dynamics_estimator.cpp.s

# Object files for target HumanDynamicsEstimator
HumanDynamicsEstimator_OBJECTS = \
"CMakeFiles/HumanDynamicsEstimator.dir/HumanDynamicsEstimator.cpp.o" \
"CMakeFiles/HumanDynamicsEstimator.dir/yarp_plugin_human_dynamics_estimator.cpp.o"

# External object files for target HumanDynamicsEstimator
HumanDynamicsEstimator_EXTERNAL_OBJECTS =

lib/yarp/HumanDynamicsEstimator.so: devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/HumanDynamicsEstimator.cpp.o
lib/yarp/HumanDynamicsEstimator.so: devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/yarp_plugin_human_dynamics_estimator.cpp.o
lib/yarp/HumanDynamicsEstimator.so: devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/build.make
lib/yarp/HumanDynamicsEstimator.so: /home/fabiana/repos/robotology-superbuild/build/install/lib/libYARP_dev.so.3.8.1
lib/yarp/HumanDynamicsEstimator.so: /home/fabiana/repos/robotology-superbuild/build/install/lib/libYARP_init.so.3.8.1
lib/yarp/HumanDynamicsEstimator.so: /usr/lib/lib/libidyntree-estimation.so
lib/yarp/HumanDynamicsEstimator.so: /home/fabiana/repos/robotology-superbuild/build/install/lib/libYARP_math.so.3.8.1
lib/yarp/HumanDynamicsEstimator.so: /home/fabiana/repos/robotology-superbuild/build/install/lib/libYARP_sig.so.3.8.1
lib/yarp/HumanDynamicsEstimator.so: /home/fabiana/repos/robotology-superbuild/build/install/lib/libYARP_os.so.3.8.1
lib/yarp/HumanDynamicsEstimator.so: /usr/lib/lib/libidyntree-modelio.so
lib/yarp/HumanDynamicsEstimator.so: /usr/lib/lib/libidyntree-model.so
lib/yarp/HumanDynamicsEstimator.so: /usr/lib/lib/libidyntree-modelio-xml.so
lib/yarp/HumanDynamicsEstimator.so: /usr/lib/lib/libidyntree-core.so
lib/yarp/HumanDynamicsEstimator.so: devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/fabiana/iCub/human-dynamics-estimation-vision2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared module ../../lib/yarp/HumanDynamicsEstimator.so"
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/HumanDynamicsEstimator && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/HumanDynamicsEstimator.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/build: lib/yarp/HumanDynamicsEstimator.so
.PHONY : devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/build

devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/clean:
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/HumanDynamicsEstimator && $(CMAKE_COMMAND) -P CMakeFiles/HumanDynamicsEstimator.dir/cmake_clean.cmake
.PHONY : devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/clean

devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/depend:
	cd /home/fabiana/iCub/human-dynamics-estimation-vision2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fabiana/iCub/human-dynamics-estimation-vision2 /home/fabiana/iCub/human-dynamics-estimation-vision2/devices/HumanDynamicsEstimator /home/fabiana/iCub/human-dynamics-estimation-vision2/build /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/HumanDynamicsEstimator /home/fabiana/iCub/human-dynamics-estimation-vision2/build/devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : devices/HumanDynamicsEstimator/CMakeFiles/HumanDynamicsEstimator.dir/depend

