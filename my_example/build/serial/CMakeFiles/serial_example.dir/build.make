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

# Include any dependencies generated for this target.
include serial/CMakeFiles/serial_example.dir/depend.make

# Include the progress variables for this target.
include serial/CMakeFiles/serial_example.dir/progress.make

# Include the compile flags for this target's objects.
include serial/CMakeFiles/serial_example.dir/flags.make

serial/CMakeFiles/serial_example.dir/examples/serial_example.cc.o: serial/CMakeFiles/serial_example.dir/flags.make
serial/CMakeFiles/serial_example.dir/examples/serial_example.cc.o: /home/furkan/Workspaces/my_example/src/serial/examples/serial_example.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/furkan/Workspaces/my_example/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object serial/CMakeFiles/serial_example.dir/examples/serial_example.cc.o"
	cd /home/furkan/Workspaces/my_example/build/serial && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/serial_example.dir/examples/serial_example.cc.o -c /home/furkan/Workspaces/my_example/src/serial/examples/serial_example.cc

serial/CMakeFiles/serial_example.dir/examples/serial_example.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serial_example.dir/examples/serial_example.cc.i"
	cd /home/furkan/Workspaces/my_example/build/serial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/furkan/Workspaces/my_example/src/serial/examples/serial_example.cc > CMakeFiles/serial_example.dir/examples/serial_example.cc.i

serial/CMakeFiles/serial_example.dir/examples/serial_example.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serial_example.dir/examples/serial_example.cc.s"
	cd /home/furkan/Workspaces/my_example/build/serial && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/furkan/Workspaces/my_example/src/serial/examples/serial_example.cc -o CMakeFiles/serial_example.dir/examples/serial_example.cc.s

# Object files for target serial_example
serial_example_OBJECTS = \
"CMakeFiles/serial_example.dir/examples/serial_example.cc.o"

# External object files for target serial_example
serial_example_EXTERNAL_OBJECTS =

/home/furkan/Workspaces/my_example/devel/lib/serial/serial_example: serial/CMakeFiles/serial_example.dir/examples/serial_example.cc.o
/home/furkan/Workspaces/my_example/devel/lib/serial/serial_example: serial/CMakeFiles/serial_example.dir/build.make
/home/furkan/Workspaces/my_example/devel/lib/serial/serial_example: /home/furkan/Workspaces/my_example/devel/lib/libserial.so
/home/furkan/Workspaces/my_example/devel/lib/serial/serial_example: serial/CMakeFiles/serial_example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/furkan/Workspaces/my_example/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/furkan/Workspaces/my_example/devel/lib/serial/serial_example"
	cd /home/furkan/Workspaces/my_example/build/serial && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/serial_example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
serial/CMakeFiles/serial_example.dir/build: /home/furkan/Workspaces/my_example/devel/lib/serial/serial_example

.PHONY : serial/CMakeFiles/serial_example.dir/build

serial/CMakeFiles/serial_example.dir/clean:
	cd /home/furkan/Workspaces/my_example/build/serial && $(CMAKE_COMMAND) -P CMakeFiles/serial_example.dir/cmake_clean.cmake
.PHONY : serial/CMakeFiles/serial_example.dir/clean

serial/CMakeFiles/serial_example.dir/depend:
	cd /home/furkan/Workspaces/my_example/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/furkan/Workspaces/my_example/src /home/furkan/Workspaces/my_example/src/serial /home/furkan/Workspaces/my_example/build /home/furkan/Workspaces/my_example/build/serial /home/furkan/Workspaces/my_example/build/serial/CMakeFiles/serial_example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : serial/CMakeFiles/serial_example.dir/depend
