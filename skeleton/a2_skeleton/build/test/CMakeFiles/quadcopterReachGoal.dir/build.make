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
CMAKE_SOURCE_DIR = /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a2_skeleton

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a2_skeleton/build

# Include any dependencies generated for this target.
include test/CMakeFiles/quadcopterReachGoal.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/quadcopterReachGoal.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/quadcopterReachGoal.dir/flags.make

test/CMakeFiles/quadcopterReachGoal.dir/quadcopterReachGoal.cpp.o: test/CMakeFiles/quadcopterReachGoal.dir/flags.make
test/CMakeFiles/quadcopterReachGoal.dir/quadcopterReachGoal.cpp.o: ../test/quadcopterReachGoal.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a2_skeleton/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/quadcopterReachGoal.dir/quadcopterReachGoal.cpp.o"
	cd /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a2_skeleton/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/quadcopterReachGoal.dir/quadcopterReachGoal.cpp.o -c /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a2_skeleton/test/quadcopterReachGoal.cpp

test/CMakeFiles/quadcopterReachGoal.dir/quadcopterReachGoal.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadcopterReachGoal.dir/quadcopterReachGoal.cpp.i"
	cd /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a2_skeleton/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a2_skeleton/test/quadcopterReachGoal.cpp > CMakeFiles/quadcopterReachGoal.dir/quadcopterReachGoal.cpp.i

test/CMakeFiles/quadcopterReachGoal.dir/quadcopterReachGoal.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadcopterReachGoal.dir/quadcopterReachGoal.cpp.s"
	cd /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a2_skeleton/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a2_skeleton/test/quadcopterReachGoal.cpp -o CMakeFiles/quadcopterReachGoal.dir/quadcopterReachGoal.cpp.s

# Object files for target quadcopterReachGoal
quadcopterReachGoal_OBJECTS = \
"CMakeFiles/quadcopterReachGoal.dir/quadcopterReachGoal.cpp.o"

# External object files for target quadcopterReachGoal
quadcopterReachGoal_EXTERNAL_OBJECTS =

test/quadcopterReachGoal: test/CMakeFiles/quadcopterReachGoal.dir/quadcopterReachGoal.cpp.o
test/quadcopterReachGoal: test/CMakeFiles/quadcopterReachGoal.dir/build.make
test/quadcopterReachGoal: /usr/src/gtest/lib/libgtest.a
test/quadcopterReachGoal: test/libstudent_lib.a
test/quadcopterReachGoal: test/CMakeFiles/quadcopterReachGoal.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a2_skeleton/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable quadcopterReachGoal"
	cd /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a2_skeleton/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/quadcopterReachGoal.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/quadcopterReachGoal.dir/build: test/quadcopterReachGoal

.PHONY : test/CMakeFiles/quadcopterReachGoal.dir/build

test/CMakeFiles/quadcopterReachGoal.dir/clean:
	cd /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a2_skeleton/build/test && $(CMAKE_COMMAND) -P CMakeFiles/quadcopterReachGoal.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/quadcopterReachGoal.dir/clean

test/CMakeFiles/quadcopterReachGoal.dir/depend:
	cd /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a2_skeleton/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a2_skeleton /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a2_skeleton/test /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a2_skeleton/build /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a2_skeleton/build/test /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a2_skeleton/build/test/CMakeFiles/quadcopterReachGoal.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/quadcopterReachGoal.dir/depend

