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
include CMakeFiles/pcd_normals.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pcd_normals.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pcd_normals.dir/flags.make

CMakeFiles/pcd_normals.dir/pcd_normals.cpp.o: CMakeFiles/pcd_normals.dir/flags.make
CMakeFiles/pcd_normals.dir/pcd_normals.cpp.o: ../pcd_normals.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /panfs/panasas1/users/meganrenshaw19_gmail.com/code/kinect_grabber/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/pcd_normals.dir/pcd_normals.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/pcd_normals.dir/pcd_normals.cpp.o -c /panfs/panasas1/users/meganrenshaw19_gmail.com/code/kinect_grabber/pcd_normals.cpp

CMakeFiles/pcd_normals.dir/pcd_normals.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pcd_normals.dir/pcd_normals.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /panfs/panasas1/users/meganrenshaw19_gmail.com/code/kinect_grabber/pcd_normals.cpp > CMakeFiles/pcd_normals.dir/pcd_normals.cpp.i

CMakeFiles/pcd_normals.dir/pcd_normals.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pcd_normals.dir/pcd_normals.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /panfs/panasas1/users/meganrenshaw19_gmail.com/code/kinect_grabber/pcd_normals.cpp -o CMakeFiles/pcd_normals.dir/pcd_normals.cpp.s

CMakeFiles/pcd_normals.dir/pcd_normals.cpp.o.requires:
.PHONY : CMakeFiles/pcd_normals.dir/pcd_normals.cpp.o.requires

CMakeFiles/pcd_normals.dir/pcd_normals.cpp.o.provides: CMakeFiles/pcd_normals.dir/pcd_normals.cpp.o.requires
	$(MAKE) -f CMakeFiles/pcd_normals.dir/build.make CMakeFiles/pcd_normals.dir/pcd_normals.cpp.o.provides.build
.PHONY : CMakeFiles/pcd_normals.dir/pcd_normals.cpp.o.provides

CMakeFiles/pcd_normals.dir/pcd_normals.cpp.o.provides.build: CMakeFiles/pcd_normals.dir/pcd_normals.cpp.o
.PHONY : CMakeFiles/pcd_normals.dir/pcd_normals.cpp.o.provides.build

# Object files for target pcd_normals
pcd_normals_OBJECTS = \
"CMakeFiles/pcd_normals.dir/pcd_normals.cpp.o"

# External object files for target pcd_normals
pcd_normals_EXTERNAL_OBJECTS =

pcd_normals: CMakeFiles/pcd_normals.dir/pcd_normals.cpp.o
pcd_normals: /usr/lib/libpcl_io.so
pcd_normals: /usr/lib/libpcl_common.so
pcd_normals: /usr/lib/libpcl_kdtree.so
pcd_normals: /usr/lib/libpcl_keypoints.so
pcd_normals: /usr/lib/libpcl_filters.so
pcd_normals: /usr/lib/libpcl_range_image.so
pcd_normals: /usr/lib/libpcl_registration.so
pcd_normals: /usr/lib/libpcl_sample_consensus.so
pcd_normals: /usr/lib/libpcl_segmentation.so
pcd_normals: /usr/lib/libpcl_features.so
pcd_normals: /usr/lib/libpcl_surface.so
pcd_normals: /usr/lib/libpcl_octree.so
pcd_normals: /usr/lib/libpcl_visualization.so
pcd_normals: /usr/lib/libGLU.so
pcd_normals: /usr/lib/libGL.so
pcd_normals: /usr/lib/libSM.so
pcd_normals: /usr/lib/libICE.so
pcd_normals: /usr/lib/libX11.so
pcd_normals: /usr/lib/libXext.so
pcd_normals: /usr/lib/libglut.so
pcd_normals: /usr/lib/libXmu.so
pcd_normals: /usr/lib/libXi.so
pcd_normals: CMakeFiles/pcd_normals.dir/build.make
pcd_normals: CMakeFiles/pcd_normals.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable pcd_normals"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pcd_normals.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pcd_normals.dir/build: pcd_normals
.PHONY : CMakeFiles/pcd_normals.dir/build

CMakeFiles/pcd_normals.dir/requires: CMakeFiles/pcd_normals.dir/pcd_normals.cpp.o.requires
.PHONY : CMakeFiles/pcd_normals.dir/requires

CMakeFiles/pcd_normals.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pcd_normals.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pcd_normals.dir/clean

CMakeFiles/pcd_normals.dir/depend:
	cd /panfs/panasas1/users/meganrenshaw19_gmail.com/code/kinect_grabber/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /panfs/panasas1/users/meganrenshaw19_gmail.com/code/kinect_grabber /panfs/panasas1/users/meganrenshaw19_gmail.com/code/kinect_grabber /panfs/panasas1/users/meganrenshaw19_gmail.com/code/kinect_grabber/build /panfs/panasas1/users/meganrenshaw19_gmail.com/code/kinect_grabber/build /panfs/panasas1/users/meganrenshaw19_gmail.com/code/kinect_grabber/build/CMakeFiles/pcd_normals.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pcd_normals.dir/depend

