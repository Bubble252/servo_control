# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ubuntu/桌面/FT_servo/FTServo_Linux/examples/SMS_STS/self_define

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/桌面/FT_servo/FTServo_Linux/examples/SMS_STS/self_define

# Include any dependencies generated for this target.
include CMakeFiles/self_define.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/self_define.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/self_define.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/self_define.dir/flags.make

CMakeFiles/self_define.dir/main.cpp.o: CMakeFiles/self_define.dir/flags.make
CMakeFiles/self_define.dir/main.cpp.o: main.cpp
CMakeFiles/self_define.dir/main.cpp.o: CMakeFiles/self_define.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/桌面/FT_servo/FTServo_Linux/examples/SMS_STS/self_define/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/self_define.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/self_define.dir/main.cpp.o -MF CMakeFiles/self_define.dir/main.cpp.o.d -o CMakeFiles/self_define.dir/main.cpp.o -c /home/ubuntu/桌面/FT_servo/FTServo_Linux/examples/SMS_STS/self_define/main.cpp

CMakeFiles/self_define.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/self_define.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/桌面/FT_servo/FTServo_Linux/examples/SMS_STS/self_define/main.cpp > CMakeFiles/self_define.dir/main.cpp.i

CMakeFiles/self_define.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/self_define.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/桌面/FT_servo/FTServo_Linux/examples/SMS_STS/self_define/main.cpp -o CMakeFiles/self_define.dir/main.cpp.s

CMakeFiles/self_define.dir/servo.cpp.o: CMakeFiles/self_define.dir/flags.make
CMakeFiles/self_define.dir/servo.cpp.o: servo.cpp
CMakeFiles/self_define.dir/servo.cpp.o: CMakeFiles/self_define.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/桌面/FT_servo/FTServo_Linux/examples/SMS_STS/self_define/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/self_define.dir/servo.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/self_define.dir/servo.cpp.o -MF CMakeFiles/self_define.dir/servo.cpp.o.d -o CMakeFiles/self_define.dir/servo.cpp.o -c /home/ubuntu/桌面/FT_servo/FTServo_Linux/examples/SMS_STS/self_define/servo.cpp

CMakeFiles/self_define.dir/servo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/self_define.dir/servo.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/桌面/FT_servo/FTServo_Linux/examples/SMS_STS/self_define/servo.cpp > CMakeFiles/self_define.dir/servo.cpp.i

CMakeFiles/self_define.dir/servo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/self_define.dir/servo.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/桌面/FT_servo/FTServo_Linux/examples/SMS_STS/self_define/servo.cpp -o CMakeFiles/self_define.dir/servo.cpp.s

# Object files for target self_define
self_define_OBJECTS = \
"CMakeFiles/self_define.dir/main.cpp.o" \
"CMakeFiles/self_define.dir/servo.cpp.o"

# External object files for target self_define
self_define_EXTERNAL_OBJECTS =

self_define: CMakeFiles/self_define.dir/main.cpp.o
self_define: CMakeFiles/self_define.dir/servo.cpp.o
self_define: CMakeFiles/self_define.dir/build.make
self_define: /home/ubuntu/桌面/FT_servo/FTServo_Linux/src/libSCServo.a
self_define: CMakeFiles/self_define.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/桌面/FT_servo/FTServo_Linux/examples/SMS_STS/self_define/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable self_define"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/self_define.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/self_define.dir/build: self_define
.PHONY : CMakeFiles/self_define.dir/build

CMakeFiles/self_define.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/self_define.dir/cmake_clean.cmake
.PHONY : CMakeFiles/self_define.dir/clean

CMakeFiles/self_define.dir/depend:
	cd /home/ubuntu/桌面/FT_servo/FTServo_Linux/examples/SMS_STS/self_define && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/桌面/FT_servo/FTServo_Linux/examples/SMS_STS/self_define /home/ubuntu/桌面/FT_servo/FTServo_Linux/examples/SMS_STS/self_define /home/ubuntu/桌面/FT_servo/FTServo_Linux/examples/SMS_STS/self_define /home/ubuntu/桌面/FT_servo/FTServo_Linux/examples/SMS_STS/self_define /home/ubuntu/桌面/FT_servo/FTServo_Linux/examples/SMS_STS/self_define/CMakeFiles/self_define.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/self_define.dir/depend

