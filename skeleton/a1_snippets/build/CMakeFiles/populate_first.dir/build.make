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
CMAKE_SOURCE_DIR = /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a1_snippets

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a1_snippets/build

# Include any dependencies generated for this target.
include CMakeFiles/populate_first.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/populate_first.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/populate_first.dir/flags.make

CMakeFiles/populate_first.dir/populate_first.cpp.o: CMakeFiles/populate_first.dir/flags.make
CMakeFiles/populate_first.dir/populate_first.cpp.o: ../populate_first.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a1_snippets/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/populate_first.dir/populate_first.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/populate_first.dir/populate_first.cpp.o -c /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a1_snippets/populate_first.cpp

CMakeFiles/populate_first.dir/populate_first.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/populate_first.dir/populate_first.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a1_snippets/populate_first.cpp > CMakeFiles/populate_first.dir/populate_first.cpp.i

CMakeFiles/populate_first.dir/populate_first.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/populate_first.dir/populate_first.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a1_snippets/populate_first.cpp -o CMakeFiles/populate_first.dir/populate_first.cpp.s

# Object files for target populate_first
populate_first_OBJECTS = \
"CMakeFiles/populate_first.dir/populate_first.cpp.o"

# External object files for target populate_first
populate_first_EXTERNAL_OBJECTS =

populate_first: CMakeFiles/populate_first.dir/populate_first.cpp.o
populate_first: CMakeFiles/populate_first.dir/build.make
populate_first: CMakeFiles/populate_first.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a1_snippets/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable populate_first"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/populate_first.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/populate_first.dir/build: populate_first

.PHONY : CMakeFiles/populate_first.dir/build

CMakeFiles/populate_first.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/populate_first.dir/cmake_clean.cmake
.PHONY : CMakeFiles/populate_first.dir/clean

CMakeFiles/populate_first.dir/depend:
	cd /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a1_snippets/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a1_snippets /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a1_snippets /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a1_snippets/build /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a1_snippets/build /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a1_snippets/build/CMakeFiles/populate_first.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/populate_first.dir/depend

