# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.15

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\JetBrains\CLion 2019.3.5\bin\cmake\win\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\JetBrains\CLion 2019.3.5\bin\cmake\win\bin\cmake.exe" -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\cyril\CLionProjects\AntikolSys_Rollstuhl_Curtis\AntikolSys_Rollstuhl_Curtis

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\cyril\CLionProjects\AntikolSys_Rollstuhl_Curtis\AntikolSys_Rollstuhl_Curtis\cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/mega_Canbus.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mega_Canbus.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mega_Canbus.dir/flags.make

CMakeFiles/mega_Canbus.dir/C_/Program_Files_(x86)/Arduino/libraries/Canbus/Canbus.cpp.obj: CMakeFiles/mega_Canbus.dir/flags.make
CMakeFiles/mega_Canbus.dir/C_/Program_Files_(x86)/Arduino/libraries/Canbus/Canbus.cpp.obj: C:/Program\ Files\ (x86)/Arduino/libraries/Canbus/Canbus.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\cyril\CLionProjects\AntikolSys_Rollstuhl_Curtis\AntikolSys_Rollstuhl_Curtis\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mega_Canbus.dir/C_/Program_Files_(x86)/Arduino/libraries/Canbus/Canbus.cpp.obj"
	C:\PROGRA~2\Arduino\hardware\tools\avr\bin\AVR-G_~1.EXE  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\mega_Canbus.dir\C_\Program_Files_(x86)\Arduino\libraries\Canbus\Canbus.cpp.obj -c "C:\Program Files (x86)\Arduino\libraries\Canbus\Canbus.cpp"

CMakeFiles/mega_Canbus.dir/C_/Program_Files_(x86)/Arduino/libraries/Canbus/Canbus.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mega_Canbus.dir/C_/Program_Files_(x86)/Arduino/libraries/Canbus/Canbus.cpp.i"
	C:\PROGRA~2\Arduino\hardware\tools\avr\bin\AVR-G_~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "C:\Program Files (x86)\Arduino\libraries\Canbus\Canbus.cpp" > CMakeFiles\mega_Canbus.dir\C_\Program_Files_(x86)\Arduino\libraries\Canbus\Canbus.cpp.i

CMakeFiles/mega_Canbus.dir/C_/Program_Files_(x86)/Arduino/libraries/Canbus/Canbus.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mega_Canbus.dir/C_/Program_Files_(x86)/Arduino/libraries/Canbus/Canbus.cpp.s"
	C:\PROGRA~2\Arduino\hardware\tools\avr\bin\AVR-G_~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "C:\Program Files (x86)\Arduino\libraries\Canbus\Canbus.cpp" -o CMakeFiles\mega_Canbus.dir\C_\Program_Files_(x86)\Arduino\libraries\Canbus\Canbus.cpp.s

# Object files for target mega_Canbus
mega_Canbus_OBJECTS = \
"CMakeFiles/mega_Canbus.dir/C_/Program_Files_(x86)/Arduino/libraries/Canbus/Canbus.cpp.obj"

# External object files for target mega_Canbus
mega_Canbus_EXTERNAL_OBJECTS =

libmega_Canbus.a: CMakeFiles/mega_Canbus.dir/C_/Program_Files_(x86)/Arduino/libraries/Canbus/Canbus.cpp.obj
libmega_Canbus.a: CMakeFiles/mega_Canbus.dir/build.make
libmega_Canbus.a: CMakeFiles/mega_Canbus.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\Users\cyril\CLionProjects\AntikolSys_Rollstuhl_Curtis\AntikolSys_Rollstuhl_Curtis\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libmega_Canbus.a"
	$(CMAKE_COMMAND) -P CMakeFiles\mega_Canbus.dir\cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\mega_Canbus.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mega_Canbus.dir/build: libmega_Canbus.a

.PHONY : CMakeFiles/mega_Canbus.dir/build

CMakeFiles/mega_Canbus.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\mega_Canbus.dir\cmake_clean.cmake
.PHONY : CMakeFiles/mega_Canbus.dir/clean

CMakeFiles/mega_Canbus.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\cyril\CLionProjects\AntikolSys_Rollstuhl_Curtis\AntikolSys_Rollstuhl_Curtis C:\Users\cyril\CLionProjects\AntikolSys_Rollstuhl_Curtis\AntikolSys_Rollstuhl_Curtis C:\Users\cyril\CLionProjects\AntikolSys_Rollstuhl_Curtis\AntikolSys_Rollstuhl_Curtis\cmake-build-debug C:\Users\cyril\CLionProjects\AntikolSys_Rollstuhl_Curtis\AntikolSys_Rollstuhl_Curtis\cmake-build-debug C:\Users\cyril\CLionProjects\AntikolSys_Rollstuhl_Curtis\AntikolSys_Rollstuhl_Curtis\cmake-build-debug\CMakeFiles\mega_Canbus.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mega_Canbus.dir/depend

