# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/demphi/ros/quad_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/demphi/ros/quad_ws/build

# Include any dependencies generated for this target.
include main_controller/CMakeFiles/remap.dir/depend.make

# Include the progress variables for this target.
include main_controller/CMakeFiles/remap.dir/progress.make

# Include the compile flags for this target's objects.
include main_controller/CMakeFiles/remap.dir/flags.make

main_controller/CMakeFiles/remap.dir/App/remap_gazebo.cpp.o: main_controller/CMakeFiles/remap.dir/flags.make
main_controller/CMakeFiles/remap.dir/App/remap_gazebo.cpp.o: /home/demphi/ros/quad_ws/src/main_controller/App/remap_gazebo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/demphi/ros/quad_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object main_controller/CMakeFiles/remap.dir/App/remap_gazebo.cpp.o"
	cd /home/demphi/ros/quad_ws/build/main_controller && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/remap.dir/App/remap_gazebo.cpp.o -c /home/demphi/ros/quad_ws/src/main_controller/App/remap_gazebo.cpp

main_controller/CMakeFiles/remap.dir/App/remap_gazebo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/remap.dir/App/remap_gazebo.cpp.i"
	cd /home/demphi/ros/quad_ws/build/main_controller && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/demphi/ros/quad_ws/src/main_controller/App/remap_gazebo.cpp > CMakeFiles/remap.dir/App/remap_gazebo.cpp.i

main_controller/CMakeFiles/remap.dir/App/remap_gazebo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/remap.dir/App/remap_gazebo.cpp.s"
	cd /home/demphi/ros/quad_ws/build/main_controller && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/demphi/ros/quad_ws/src/main_controller/App/remap_gazebo.cpp -o CMakeFiles/remap.dir/App/remap_gazebo.cpp.s

# Object files for target remap
remap_OBJECTS = \
"CMakeFiles/remap.dir/App/remap_gazebo.cpp.o"

# External object files for target remap
remap_EXTERNAL_OBJECTS =

/home/demphi/ros/quad_ws/devel/lib/main_controller/remap: main_controller/CMakeFiles/remap.dir/App/remap_gazebo.cpp.o
/home/demphi/ros/quad_ws/devel/lib/main_controller/remap: main_controller/CMakeFiles/remap.dir/build.make
/home/demphi/ros/quad_ws/devel/lib/main_controller/remap: /opt/ros/melodic/lib/libroscpp.so
/home/demphi/ros/quad_ws/devel/lib/main_controller/remap: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/demphi/ros/quad_ws/devel/lib/main_controller/remap: /opt/ros/melodic/lib/librosconsole.so
/home/demphi/ros/quad_ws/devel/lib/main_controller/remap: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/demphi/ros/quad_ws/devel/lib/main_controller/remap: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/demphi/ros/quad_ws/devel/lib/main_controller/remap: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/demphi/ros/quad_ws/devel/lib/main_controller/remap: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/demphi/ros/quad_ws/devel/lib/main_controller/remap: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/demphi/ros/quad_ws/devel/lib/main_controller/remap: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/demphi/ros/quad_ws/devel/lib/main_controller/remap: /opt/ros/melodic/lib/librostime.so
/home/demphi/ros/quad_ws/devel/lib/main_controller/remap: /opt/ros/melodic/lib/libcpp_common.so
/home/demphi/ros/quad_ws/devel/lib/main_controller/remap: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/demphi/ros/quad_ws/devel/lib/main_controller/remap: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/demphi/ros/quad_ws/devel/lib/main_controller/remap: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/demphi/ros/quad_ws/devel/lib/main_controller/remap: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/demphi/ros/quad_ws/devel/lib/main_controller/remap: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/demphi/ros/quad_ws/devel/lib/main_controller/remap: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/demphi/ros/quad_ws/devel/lib/main_controller/remap: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/demphi/ros/quad_ws/devel/lib/main_controller/remap: main_controller/CMakeFiles/remap.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/demphi/ros/quad_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/demphi/ros/quad_ws/devel/lib/main_controller/remap"
	cd /home/demphi/ros/quad_ws/build/main_controller && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/remap.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
main_controller/CMakeFiles/remap.dir/build: /home/demphi/ros/quad_ws/devel/lib/main_controller/remap

.PHONY : main_controller/CMakeFiles/remap.dir/build

main_controller/CMakeFiles/remap.dir/clean:
	cd /home/demphi/ros/quad_ws/build/main_controller && $(CMAKE_COMMAND) -P CMakeFiles/remap.dir/cmake_clean.cmake
.PHONY : main_controller/CMakeFiles/remap.dir/clean

main_controller/CMakeFiles/remap.dir/depend:
	cd /home/demphi/ros/quad_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/demphi/ros/quad_ws/src /home/demphi/ros/quad_ws/src/main_controller /home/demphi/ros/quad_ws/build /home/demphi/ros/quad_ws/build/main_controller /home/demphi/ros/quad_ws/build/main_controller/CMakeFiles/remap.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : main_controller/CMakeFiles/remap.dir/depend

