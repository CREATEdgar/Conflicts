# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_COMMAND = /home/edgar/Programs/clion-2018.1/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/edgar/Programs/clion-2018.1/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/edgar/CLionProjects/Statistics

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/edgar/CLionProjects/Statistics/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/Statistics.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Statistics.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Statistics.dir/flags.make

CMakeFiles/Statistics.dir/main.cpp.o: CMakeFiles/Statistics.dir/flags.make
CMakeFiles/Statistics.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/edgar/CLionProjects/Statistics/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Statistics.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Statistics.dir/main.cpp.o -c /home/edgar/CLionProjects/Statistics/main.cpp

CMakeFiles/Statistics.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Statistics.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/edgar/CLionProjects/Statistics/main.cpp > CMakeFiles/Statistics.dir/main.cpp.i

CMakeFiles/Statistics.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Statistics.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/edgar/CLionProjects/Statistics/main.cpp -o CMakeFiles/Statistics.dir/main.cpp.s

CMakeFiles/Statistics.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/Statistics.dir/main.cpp.o.requires

CMakeFiles/Statistics.dir/main.cpp.o.provides: CMakeFiles/Statistics.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/Statistics.dir/build.make CMakeFiles/Statistics.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/Statistics.dir/main.cpp.o.provides

CMakeFiles/Statistics.dir/main.cpp.o.provides.build: CMakeFiles/Statistics.dir/main.cpp.o


# Object files for target Statistics
Statistics_OBJECTS = \
"CMakeFiles/Statistics.dir/main.cpp.o"

# External object files for target Statistics
Statistics_EXTERNAL_OBJECTS =

Statistics: CMakeFiles/Statistics.dir/main.cpp.o
Statistics: CMakeFiles/Statistics.dir/build.make
Statistics: CMakeFiles/Statistics.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/edgar/CLionProjects/Statistics/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Statistics"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Statistics.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Statistics.dir/build: Statistics

.PHONY : CMakeFiles/Statistics.dir/build

CMakeFiles/Statistics.dir/requires: CMakeFiles/Statistics.dir/main.cpp.o.requires

.PHONY : CMakeFiles/Statistics.dir/requires

CMakeFiles/Statistics.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Statistics.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Statistics.dir/clean

CMakeFiles/Statistics.dir/depend:
	cd /home/edgar/CLionProjects/Statistics/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/edgar/CLionProjects/Statistics /home/edgar/CLionProjects/Statistics /home/edgar/CLionProjects/Statistics/cmake-build-debug /home/edgar/CLionProjects/Statistics/cmake-build-debug /home/edgar/CLionProjects/Statistics/cmake-build-debug/CMakeFiles/Statistics.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Statistics.dir/depend
