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
CMAKE_SOURCE_DIR = /home/furkan/Workspaces/my_example/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/furkan/Workspaces/my_example/build

# Utility rule file for run_tests_serial_gtest_serial-test-timer.

# Include the progress variables for this target.
include serial/tests/CMakeFiles/run_tests_serial_gtest_serial-test-timer.dir/progress.make

serial/tests/CMakeFiles/run_tests_serial_gtest_serial-test-timer:
	cd /home/furkan/Workspaces/my_example/build/serial/tests && ../../catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/catkin/cmake/test/run_tests.py /home/furkan/Workspaces/my_example/build/test_results/serial/gtest-serial-test-timer.xml "/home/furkan/Workspaces/my_example/devel/lib/serial/serial-test-timer --gtest_output=xml:/home/furkan/Workspaces/my_example/build/test_results/serial/gtest-serial-test-timer.xml"

run_tests_serial_gtest_serial-test-timer: serial/tests/CMakeFiles/run_tests_serial_gtest_serial-test-timer
run_tests_serial_gtest_serial-test-timer: serial/tests/CMakeFiles/run_tests_serial_gtest_serial-test-timer.dir/build.make

.PHONY : run_tests_serial_gtest_serial-test-timer

# Rule to build all files generated by this target.
serial/tests/CMakeFiles/run_tests_serial_gtest_serial-test-timer.dir/build: run_tests_serial_gtest_serial-test-timer

.PHONY : serial/tests/CMakeFiles/run_tests_serial_gtest_serial-test-timer.dir/build

serial/tests/CMakeFiles/run_tests_serial_gtest_serial-test-timer.dir/clean:
	cd /home/furkan/Workspaces/my_example/build/serial/tests && $(CMAKE_COMMAND) -P CMakeFiles/run_tests_serial_gtest_serial-test-timer.dir/cmake_clean.cmake
.PHONY : serial/tests/CMakeFiles/run_tests_serial_gtest_serial-test-timer.dir/clean

serial/tests/CMakeFiles/run_tests_serial_gtest_serial-test-timer.dir/depend:
	cd /home/furkan/Workspaces/my_example/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/furkan/Workspaces/my_example/src /home/furkan/Workspaces/my_example/src/serial/tests /home/furkan/Workspaces/my_example/build /home/furkan/Workspaces/my_example/build/serial/tests /home/furkan/Workspaces/my_example/build/serial/tests/CMakeFiles/run_tests_serial_gtest_serial-test-timer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : serial/tests/CMakeFiles/run_tests_serial_gtest_serial-test-timer.dir/depend

