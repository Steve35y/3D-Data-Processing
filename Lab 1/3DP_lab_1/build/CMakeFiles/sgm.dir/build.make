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
CMAKE_SOURCE_DIR = "/home/stefano/Scrivania/3DP/3D-Data-Processing/Lab 1/3DP_lab_1"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/stefano/Scrivania/3DP/3D-Data-Processing/Lab 1/3DP_lab_1/build"

# Include any dependencies generated for this target.
include CMakeFiles/sgm.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sgm.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sgm.dir/flags.make

CMakeFiles/sgm.dir/sgm.cpp.o: CMakeFiles/sgm.dir/flags.make
CMakeFiles/sgm.dir/sgm.cpp.o: ../sgm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/stefano/Scrivania/3DP/3D-Data-Processing/Lab 1/3DP_lab_1/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sgm.dir/sgm.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sgm.dir/sgm.cpp.o -c "/home/stefano/Scrivania/3DP/3D-Data-Processing/Lab 1/3DP_lab_1/sgm.cpp"

CMakeFiles/sgm.dir/sgm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sgm.dir/sgm.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/stefano/Scrivania/3DP/3D-Data-Processing/Lab 1/3DP_lab_1/sgm.cpp" > CMakeFiles/sgm.dir/sgm.cpp.i

CMakeFiles/sgm.dir/sgm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sgm.dir/sgm.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/stefano/Scrivania/3DP/3D-Data-Processing/Lab 1/3DP_lab_1/sgm.cpp" -o CMakeFiles/sgm.dir/sgm.cpp.s

CMakeFiles/sgm.dir/main.cpp.o: CMakeFiles/sgm.dir/flags.make
CMakeFiles/sgm.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/stefano/Scrivania/3DP/3D-Data-Processing/Lab 1/3DP_lab_1/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/sgm.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sgm.dir/main.cpp.o -c "/home/stefano/Scrivania/3DP/3D-Data-Processing/Lab 1/3DP_lab_1/main.cpp"

CMakeFiles/sgm.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sgm.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/stefano/Scrivania/3DP/3D-Data-Processing/Lab 1/3DP_lab_1/main.cpp" > CMakeFiles/sgm.dir/main.cpp.i

CMakeFiles/sgm.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sgm.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/stefano/Scrivania/3DP/3D-Data-Processing/Lab 1/3DP_lab_1/main.cpp" -o CMakeFiles/sgm.dir/main.cpp.s

# Object files for target sgm
sgm_OBJECTS = \
"CMakeFiles/sgm.dir/sgm.cpp.o" \
"CMakeFiles/sgm.dir/main.cpp.o"

# External object files for target sgm
sgm_EXTERNAL_OBJECTS =

sgm: CMakeFiles/sgm.dir/sgm.cpp.o
sgm: CMakeFiles/sgm.dir/main.cpp.o
sgm: CMakeFiles/sgm.dir/build.make
sgm: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
sgm: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
sgm: CMakeFiles/sgm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/stefano/Scrivania/3DP/3D-Data-Processing/Lab 1/3DP_lab_1/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable sgm"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sgm.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sgm.dir/build: sgm

.PHONY : CMakeFiles/sgm.dir/build

CMakeFiles/sgm.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sgm.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sgm.dir/clean

CMakeFiles/sgm.dir/depend:
	cd "/home/stefano/Scrivania/3DP/3D-Data-Processing/Lab 1/3DP_lab_1/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/stefano/Scrivania/3DP/3D-Data-Processing/Lab 1/3DP_lab_1" "/home/stefano/Scrivania/3DP/3D-Data-Processing/Lab 1/3DP_lab_1" "/home/stefano/Scrivania/3DP/3D-Data-Processing/Lab 1/3DP_lab_1/build" "/home/stefano/Scrivania/3DP/3D-Data-Processing/Lab 1/3DP_lab_1/build" "/home/stefano/Scrivania/3DP/3D-Data-Processing/Lab 1/3DP_lab_1/build/CMakeFiles/sgm.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/sgm.dir/depend

