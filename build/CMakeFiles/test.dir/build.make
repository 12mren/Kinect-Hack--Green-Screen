# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /panfs/panasas1/users/meganrenshaw19_gmail.com/code/kinect_grabber

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /panfs/panasas1/users/meganrenshaw19_gmail.com/code/kinect_grabber/build

# Include any dependencies generated for this target.
include CMakeFiles/test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test.dir/flags.make

CMakeFiles/test.dir/test.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/test.cpp.o: ../test.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /panfs/panasas1/users/meganrenshaw19_gmail.com/code/kinect_grabber/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test.dir/test.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test.dir/test.cpp.o -c /panfs/panasas1/users/meganrenshaw19_gmail.com/code/kinect_grabber/test.cpp

CMakeFiles/test.dir/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/test.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /panfs/panasas1/users/meganrenshaw19_gmail.com/code/kinect_grabber/test.cpp > CMakeFiles/test.dir/test.cpp.i

CMakeFiles/test.dir/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/test.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /panfs/panasas1/users/meganrenshaw19_gmail.com/code/kinect_grabber/test.cpp -o CMakeFiles/test.dir/test.cpp.s

CMakeFiles/test.dir/test.cpp.o.requires:
.PHONY : CMakeFiles/test.dir/test.cpp.o.requires

CMakeFiles/test.dir/test.cpp.o.provides: CMakeFiles/test.dir/test.cpp.o.requires
	$(MAKE) -f CMakeFiles/test.dir/build.make CMakeFiles/test.dir/test.cpp.o.provides.build
.PHONY : CMakeFiles/test.dir/test.cpp.o.provides

CMakeFiles/test.dir/test.cpp.o.provides.build: CMakeFiles/test.dir/test.cpp.o
.PHONY : CMakeFiles/test.dir/test.cpp.o.provides.build

# Object files for target test
test_OBJECTS = \
"CMakeFiles/test.dir/test.cpp.o"

# External object files for target test
test_EXTERNAL_OBJECTS =

test: CMakeFiles/test.dir/test.cpp.o
test: /usr/lib/libpcl_io.so
test: /usr/lib/libpcl_common.so
test: /usr/lib/libpcl_kdtree.so
test: /usr/lib/libpcl_keypoints.so
test: /usr/lib/libpcl_filters.so
test: /usr/lib/libpcl_range_image.so
test: /usr/lib/libpcl_registration.so
test: /usr/lib/libpcl_sample_consensus.so
test: /usr/lib/libpcl_segmentation.so
test: /usr/lib/libpcl_features.so
test: /usr/lib/libpcl_surface.so
test: /usr/lib/libpcl_octree.so
test: /usr/lib/libpcl_visualization.so
test: /usr/lib/libGLU.so
test: /usr/lib/libGL.so
test: /usr/lib/libSM.so
test: /usr/lib/libICE.so
test: /usr/lib/libX11.so
test: /usr/lib/libXext.so
test: /usr/lib/libglut.so
test: /usr/lib/libXmu.so
test: /usr/lib/libXi.so
test: CMakeFiles/test.dir/build.make
test: CMakeFiles/test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test.dir/build: test
.PHONY : CMakeFiles/test.dir/build

CMakeFiles/test.dir/requires: CMakeFiles/test.dir/test.cpp.o.requires
.PHONY : CMakeFiles/test.dir/requires

CMakeFiles/test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test.dir/clean

CMakeFiles/test.dir/depend:
	cd /panfs/panasas1/users/meganrenshaw19_gmail.com/code/kinect_grabber/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /panfs/panasas1/users/meganrenshaw19_gmail.com/code/kinect_grabber /panfs/panasas1/users/meganrenshaw19_gmail.com/code/kinect_grabber /panfs/panasas1/users/meganrenshaw19_gmail.com/code/kinect_grabber/build /panfs/panasas1/users/meganrenshaw19_gmail.com/code/kinect_grabber/build /panfs/panasas1/users/meganrenshaw19_gmail.com/code/kinect_grabber/build/CMakeFiles/test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test.dir/depend

