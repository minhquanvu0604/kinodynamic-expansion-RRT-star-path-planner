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
CMAKE_SOURCE_DIR = /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a3_support

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a3_support/build

# Include any dependencies generated for this target.
include CMakeFiles/a3_support_logger.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/a3_support_logger.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/a3_support_logger.dir/flags.make

CMakeFiles/a3_support_logger.dir/src/logger.cpp.o: CMakeFiles/a3_support_logger.dir/flags.make
CMakeFiles/a3_support_logger.dir/src/logger.cpp.o: ../src/logger.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a3_support/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/a3_support_logger.dir/src/logger.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/a3_support_logger.dir/src/logger.cpp.o -c /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a3_support/src/logger.cpp

CMakeFiles/a3_support_logger.dir/src/logger.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a3_support_logger.dir/src/logger.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a3_support/src/logger.cpp > CMakeFiles/a3_support_logger.dir/src/logger.cpp.i

CMakeFiles/a3_support_logger.dir/src/logger.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a3_support_logger.dir/src/logger.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a3_support/src/logger.cpp -o CMakeFiles/a3_support_logger.dir/src/logger.cpp.s

# Object files for target a3_support_logger
a3_support_logger_OBJECTS = \
"CMakeFiles/a3_support_logger.dir/src/logger.cpp.o"

# External object files for target a3_support_logger
a3_support_logger_EXTERNAL_OBJECTS =

devel/lib/liba3_support_logger.so: CMakeFiles/a3_support_logger.dir/src/logger.cpp.o
devel/lib/liba3_support_logger.so: CMakeFiles/a3_support_logger.dir/build.make
devel/lib/liba3_support_logger.so: CMakeFiles/a3_support_logger.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a3_support/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library devel/lib/liba3_support_logger.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/a3_support_logger.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/a3_support_logger.dir/build: devel/lib/liba3_support_logger.so

.PHONY : CMakeFiles/a3_support_logger.dir/build

CMakeFiles/a3_support_logger.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/a3_support_logger.dir/cmake_clean.cmake
.PHONY : CMakeFiles/a3_support_logger.dir/clean

CMakeFiles/a3_support_logger.dir/depend:
	cd /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a3_support/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a3_support /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a3_support /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a3_support/build /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a3_support/build /home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a3_support/build/CMakeFiles/a3_support_logger.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/a3_support_logger.dir/depend

