# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.30

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
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.30.0/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.30.0/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/Users/shizuku/Documents/Coding Sōko/raytracer"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/Users/shizuku/Documents/Coding Sōko/raytracer/build"

# Include any dependencies generated for this target.
include CMakeFiles/raytracer.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/raytracer.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/raytracer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/raytracer.dir/flags.make

CMakeFiles/raytracer.dir/source/main.cpp.o: CMakeFiles/raytracer.dir/flags.make
CMakeFiles/raytracer.dir/source/main.cpp.o: /Users/shizuku/Documents/Coding\ Sōko/raytracer/source/main.cpp
CMakeFiles/raytracer.dir/source/main.cpp.o: CMakeFiles/raytracer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir="/Users/shizuku/Documents/Coding Sōko/raytracer/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/raytracer.dir/source/main.cpp.o"
	/opt/homebrew/bin/g++-14 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/raytracer.dir/source/main.cpp.o -MF CMakeFiles/raytracer.dir/source/main.cpp.o.d -o CMakeFiles/raytracer.dir/source/main.cpp.o -c "/Users/shizuku/Documents/Coding Sōko/raytracer/source/main.cpp"

CMakeFiles/raytracer.dir/source/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/raytracer.dir/source/main.cpp.i"
	/opt/homebrew/bin/g++-14 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/Users/shizuku/Documents/Coding Sōko/raytracer/source/main.cpp" > CMakeFiles/raytracer.dir/source/main.cpp.i

CMakeFiles/raytracer.dir/source/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/raytracer.dir/source/main.cpp.s"
	/opt/homebrew/bin/g++-14 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/Users/shizuku/Documents/Coding Sōko/raytracer/source/main.cpp" -o CMakeFiles/raytracer.dir/source/main.cpp.s

CMakeFiles/raytracer.dir/source/shapes.cpp.o: CMakeFiles/raytracer.dir/flags.make
CMakeFiles/raytracer.dir/source/shapes.cpp.o: /Users/shizuku/Documents/Coding\ Sōko/raytracer/source/shapes.cpp
CMakeFiles/raytracer.dir/source/shapes.cpp.o: CMakeFiles/raytracer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir="/Users/shizuku/Documents/Coding Sōko/raytracer/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/raytracer.dir/source/shapes.cpp.o"
	/opt/homebrew/bin/g++-14 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/raytracer.dir/source/shapes.cpp.o -MF CMakeFiles/raytracer.dir/source/shapes.cpp.o.d -o CMakeFiles/raytracer.dir/source/shapes.cpp.o -c "/Users/shizuku/Documents/Coding Sōko/raytracer/source/shapes.cpp"

CMakeFiles/raytracer.dir/source/shapes.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/raytracer.dir/source/shapes.cpp.i"
	/opt/homebrew/bin/g++-14 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/Users/shizuku/Documents/Coding Sōko/raytracer/source/shapes.cpp" > CMakeFiles/raytracer.dir/source/shapes.cpp.i

CMakeFiles/raytracer.dir/source/shapes.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/raytracer.dir/source/shapes.cpp.s"
	/opt/homebrew/bin/g++-14 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/Users/shizuku/Documents/Coding Sōko/raytracer/source/shapes.cpp" -o CMakeFiles/raytracer.dir/source/shapes.cpp.s

CMakeFiles/raytracer.dir/source/image.cpp.o: CMakeFiles/raytracer.dir/flags.make
CMakeFiles/raytracer.dir/source/image.cpp.o: /Users/shizuku/Documents/Coding\ Sōko/raytracer/source/image.cpp
CMakeFiles/raytracer.dir/source/image.cpp.o: CMakeFiles/raytracer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir="/Users/shizuku/Documents/Coding Sōko/raytracer/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/raytracer.dir/source/image.cpp.o"
	/opt/homebrew/bin/g++-14 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/raytracer.dir/source/image.cpp.o -MF CMakeFiles/raytracer.dir/source/image.cpp.o.d -o CMakeFiles/raytracer.dir/source/image.cpp.o -c "/Users/shizuku/Documents/Coding Sōko/raytracer/source/image.cpp"

CMakeFiles/raytracer.dir/source/image.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/raytracer.dir/source/image.cpp.i"
	/opt/homebrew/bin/g++-14 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/Users/shizuku/Documents/Coding Sōko/raytracer/source/image.cpp" > CMakeFiles/raytracer.dir/source/image.cpp.i

CMakeFiles/raytracer.dir/source/image.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/raytracer.dir/source/image.cpp.s"
	/opt/homebrew/bin/g++-14 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/Users/shizuku/Documents/Coding Sōko/raytracer/source/image.cpp" -o CMakeFiles/raytracer.dir/source/image.cpp.s

CMakeFiles/raytracer.dir/source/interval.cpp.o: CMakeFiles/raytracer.dir/flags.make
CMakeFiles/raytracer.dir/source/interval.cpp.o: /Users/shizuku/Documents/Coding\ Sōko/raytracer/source/interval.cpp
CMakeFiles/raytracer.dir/source/interval.cpp.o: CMakeFiles/raytracer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir="/Users/shizuku/Documents/Coding Sōko/raytracer/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/raytracer.dir/source/interval.cpp.o"
	/opt/homebrew/bin/g++-14 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/raytracer.dir/source/interval.cpp.o -MF CMakeFiles/raytracer.dir/source/interval.cpp.o.d -o CMakeFiles/raytracer.dir/source/interval.cpp.o -c "/Users/shizuku/Documents/Coding Sōko/raytracer/source/interval.cpp"

CMakeFiles/raytracer.dir/source/interval.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/raytracer.dir/source/interval.cpp.i"
	/opt/homebrew/bin/g++-14 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/Users/shizuku/Documents/Coding Sōko/raytracer/source/interval.cpp" > CMakeFiles/raytracer.dir/source/interval.cpp.i

CMakeFiles/raytracer.dir/source/interval.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/raytracer.dir/source/interval.cpp.s"
	/opt/homebrew/bin/g++-14 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/Users/shizuku/Documents/Coding Sōko/raytracer/source/interval.cpp" -o CMakeFiles/raytracer.dir/source/interval.cpp.s

CMakeFiles/raytracer.dir/source/bounds.cpp.o: CMakeFiles/raytracer.dir/flags.make
CMakeFiles/raytracer.dir/source/bounds.cpp.o: /Users/shizuku/Documents/Coding\ Sōko/raytracer/source/bounds.cpp
CMakeFiles/raytracer.dir/source/bounds.cpp.o: CMakeFiles/raytracer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir="/Users/shizuku/Documents/Coding Sōko/raytracer/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/raytracer.dir/source/bounds.cpp.o"
	/opt/homebrew/bin/g++-14 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/raytracer.dir/source/bounds.cpp.o -MF CMakeFiles/raytracer.dir/source/bounds.cpp.o.d -o CMakeFiles/raytracer.dir/source/bounds.cpp.o -c "/Users/shizuku/Documents/Coding Sōko/raytracer/source/bounds.cpp"

CMakeFiles/raytracer.dir/source/bounds.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/raytracer.dir/source/bounds.cpp.i"
	/opt/homebrew/bin/g++-14 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/Users/shizuku/Documents/Coding Sōko/raytracer/source/bounds.cpp" > CMakeFiles/raytracer.dir/source/bounds.cpp.i

CMakeFiles/raytracer.dir/source/bounds.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/raytracer.dir/source/bounds.cpp.s"
	/opt/homebrew/bin/g++-14 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/Users/shizuku/Documents/Coding Sōko/raytracer/source/bounds.cpp" -o CMakeFiles/raytracer.dir/source/bounds.cpp.s

# Object files for target raytracer
raytracer_OBJECTS = \
"CMakeFiles/raytracer.dir/source/main.cpp.o" \
"CMakeFiles/raytracer.dir/source/shapes.cpp.o" \
"CMakeFiles/raytracer.dir/source/image.cpp.o" \
"CMakeFiles/raytracer.dir/source/interval.cpp.o" \
"CMakeFiles/raytracer.dir/source/bounds.cpp.o"

# External object files for target raytracer
raytracer_EXTERNAL_OBJECTS =

raytracer: CMakeFiles/raytracer.dir/source/main.cpp.o
raytracer: CMakeFiles/raytracer.dir/source/shapes.cpp.o
raytracer: CMakeFiles/raytracer.dir/source/image.cpp.o
raytracer: CMakeFiles/raytracer.dir/source/interval.cpp.o
raytracer: CMakeFiles/raytracer.dir/source/bounds.cpp.o
raytracer: CMakeFiles/raytracer.dir/build.make
raytracer: CMakeFiles/raytracer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir="/Users/shizuku/Documents/Coding Sōko/raytracer/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable raytracer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/raytracer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/raytracer.dir/build: raytracer
.PHONY : CMakeFiles/raytracer.dir/build

CMakeFiles/raytracer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/raytracer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/raytracer.dir/clean

CMakeFiles/raytracer.dir/depend:
	cd "/Users/shizuku/Documents/Coding Sōko/raytracer/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/Users/shizuku/Documents/Coding Sōko/raytracer" "/Users/shizuku/Documents/Coding Sōko/raytracer" "/Users/shizuku/Documents/Coding Sōko/raytracer/build" "/Users/shizuku/Documents/Coding Sōko/raytracer/build" "/Users/shizuku/Documents/Coding Sōko/raytracer/build/CMakeFiles/raytracer.dir/DependInfo.cmake" "--color=$(COLOR)"
.PHONY : CMakeFiles/raytracer.dir/depend

