# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ubuntu/thesis_mqtt/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/thesis_mqtt/build

# Include any dependencies generated for this target.
include mqtt_demo/CMakeFiles/pos_receiver.dir/depend.make

# Include the progress variables for this target.
include mqtt_demo/CMakeFiles/pos_receiver.dir/progress.make

# Include the compile flags for this target's objects.
include mqtt_demo/CMakeFiles/pos_receiver.dir/flags.make

mqtt_demo/CMakeFiles/pos_receiver.dir/clean:
	cd /home/ubuntu/thesis_mqtt/build/mqtt_demo && $(CMAKE_COMMAND) -P CMakeFiles/pos_receiver.dir/cmake_clean.cmake
.PHONY : mqtt_demo/CMakeFiles/pos_receiver.dir/clean

mqtt_demo/CMakeFiles/pos_receiver.dir/depend:
	cd /home/ubuntu/thesis_mqtt/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/thesis_mqtt/src /home/ubuntu/thesis_mqtt/src/mqtt_demo /home/ubuntu/thesis_mqtt/build /home/ubuntu/thesis_mqtt/build/mqtt_demo /home/ubuntu/thesis_mqtt/build/mqtt_demo/CMakeFiles/pos_receiver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : mqtt_demo/CMakeFiles/pos_receiver.dir/depend

