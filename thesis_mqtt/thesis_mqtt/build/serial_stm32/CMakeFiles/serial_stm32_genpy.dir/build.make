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

# Utility rule file for serial_stm32_genpy.

# Include the progress variables for this target.
include serial_stm32/CMakeFiles/serial_stm32_genpy.dir/progress.make

serial_stm32_genpy: serial_stm32/CMakeFiles/serial_stm32_genpy.dir/build.make

.PHONY : serial_stm32_genpy

# Rule to build all files generated by this target.
serial_stm32/CMakeFiles/serial_stm32_genpy.dir/build: serial_stm32_genpy

.PHONY : serial_stm32/CMakeFiles/serial_stm32_genpy.dir/build

serial_stm32/CMakeFiles/serial_stm32_genpy.dir/clean:
	cd /home/ubuntu/thesis_mqtt/build/serial_stm32 && $(CMAKE_COMMAND) -P CMakeFiles/serial_stm32_genpy.dir/cmake_clean.cmake
.PHONY : serial_stm32/CMakeFiles/serial_stm32_genpy.dir/clean

serial_stm32/CMakeFiles/serial_stm32_genpy.dir/depend:
	cd /home/ubuntu/thesis_mqtt/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/thesis_mqtt/src /home/ubuntu/thesis_mqtt/src/serial_stm32 /home/ubuntu/thesis_mqtt/build /home/ubuntu/thesis_mqtt/build/serial_stm32 /home/ubuntu/thesis_mqtt/build/serial_stm32/CMakeFiles/serial_stm32_genpy.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : serial_stm32/CMakeFiles/serial_stm32_genpy.dir/depend

