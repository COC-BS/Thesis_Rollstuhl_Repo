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

# Utility rule file for AntikolSys_Rollstuhl_Curtis-size.

# Include the progress variables for this target.
include CMakeFiles/AntikolSys_Rollstuhl_Curtis-size.dir/progress.make

CMakeFiles/AntikolSys_Rollstuhl_Curtis-size: AntikolSys_Rollstuhl_Curtis.elf
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\Users\cyril\CLionProjects\AntikolSys_Rollstuhl_Curtis\AntikolSys_Rollstuhl_Curtis\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Calculating AntikolSys_Rollstuhl_Curtis image size"
	"C:\Program Files\JetBrains\CLion 2019.3.5\bin\cmake\win\bin\cmake.exe" -DFIRMWARE_IMAGE=C:/Users/cyril/CLionProjects/AntikolSys_Rollstuhl_Curtis/AntikolSys_Rollstuhl_Curtis/cmake-build-debug/AntikolSys_Rollstuhl_Curtis.elf -DMCU=atmega2560 -DEEPROM_IMAGE=C:/Users/cyril/CLionProjects/AntikolSys_Rollstuhl_Curtis/AntikolSys_Rollstuhl_Curtis/cmake-build-debug/AntikolSys_Rollstuhl_Curtis.eep -P C:/Users/cyril/CLionProjects/AntikolSys_Rollstuhl_Curtis/AntikolSys_Rollstuhl_Curtis/cmake-build-debug/CMakeFiles/FirmwareSize.cmake

AntikolSys_Rollstuhl_Curtis-size: CMakeFiles/AntikolSys_Rollstuhl_Curtis-size
AntikolSys_Rollstuhl_Curtis-size: CMakeFiles/AntikolSys_Rollstuhl_Curtis-size.dir/build.make

.PHONY : AntikolSys_Rollstuhl_Curtis-size

# Rule to build all files generated by this target.
CMakeFiles/AntikolSys_Rollstuhl_Curtis-size.dir/build: AntikolSys_Rollstuhl_Curtis-size

.PHONY : CMakeFiles/AntikolSys_Rollstuhl_Curtis-size.dir/build

CMakeFiles/AntikolSys_Rollstuhl_Curtis-size.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\AntikolSys_Rollstuhl_Curtis-size.dir\cmake_clean.cmake
.PHONY : CMakeFiles/AntikolSys_Rollstuhl_Curtis-size.dir/clean

CMakeFiles/AntikolSys_Rollstuhl_Curtis-size.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\cyril\CLionProjects\AntikolSys_Rollstuhl_Curtis\AntikolSys_Rollstuhl_Curtis C:\Users\cyril\CLionProjects\AntikolSys_Rollstuhl_Curtis\AntikolSys_Rollstuhl_Curtis C:\Users\cyril\CLionProjects\AntikolSys_Rollstuhl_Curtis\AntikolSys_Rollstuhl_Curtis\cmake-build-debug C:\Users\cyril\CLionProjects\AntikolSys_Rollstuhl_Curtis\AntikolSys_Rollstuhl_Curtis\cmake-build-debug C:\Users\cyril\CLionProjects\AntikolSys_Rollstuhl_Curtis\AntikolSys_Rollstuhl_Curtis\cmake-build-debug\CMakeFiles\AntikolSys_Rollstuhl_Curtis-size.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/AntikolSys_Rollstuhl_Curtis-size.dir/depend

