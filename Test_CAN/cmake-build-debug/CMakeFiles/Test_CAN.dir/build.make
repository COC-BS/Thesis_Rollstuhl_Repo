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
CMAKE_SOURCE_DIR = C:\Users\cyril\CLionProjects\Test_CAN

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\cyril\CLionProjects\Test_CAN\cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/Test_CAN.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Test_CAN.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Test_CAN.dir/flags.make

Test_CAN_Test_CAN.ino.cpp: ../Test_CAN.ino
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=C:\Users\cyril\CLionProjects\Test_CAN\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Regnerating Test_CAN.ino Sketch"
	"C:\Program Files\JetBrains\CLion 2019.3.5\bin\cmake\win\bin\cmake.exe" C:/Users/cyril/CLionProjects/Test_CAN

CMakeFiles/Test_CAN.dir/Test_CAN_Test_CAN.ino.cpp.obj: CMakeFiles/Test_CAN.dir/flags.make
CMakeFiles/Test_CAN.dir/Test_CAN_Test_CAN.ino.cpp.obj: Test_CAN_Test_CAN.ino.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\cyril\CLionProjects\Test_CAN\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Test_CAN.dir/Test_CAN_Test_CAN.ino.cpp.obj"
	C:\PROGRA~2\Arduino\hardware\tools\avr\bin\AVR-G_~1.EXE  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\Test_CAN.dir\Test_CAN_Test_CAN.ino.cpp.obj -c C:\Users\cyril\CLionProjects\Test_CAN\cmake-build-debug\Test_CAN_Test_CAN.ino.cpp

CMakeFiles/Test_CAN.dir/Test_CAN_Test_CAN.ino.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Test_CAN.dir/Test_CAN_Test_CAN.ino.cpp.i"
	C:\PROGRA~2\Arduino\hardware\tools\avr\bin\AVR-G_~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\cyril\CLionProjects\Test_CAN\cmake-build-debug\Test_CAN_Test_CAN.ino.cpp > CMakeFiles\Test_CAN.dir\Test_CAN_Test_CAN.ino.cpp.i

CMakeFiles/Test_CAN.dir/Test_CAN_Test_CAN.ino.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Test_CAN.dir/Test_CAN_Test_CAN.ino.cpp.s"
	C:\PROGRA~2\Arduino\hardware\tools\avr\bin\AVR-G_~1.EXE $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\cyril\CLionProjects\Test_CAN\cmake-build-debug\Test_CAN_Test_CAN.ino.cpp -o CMakeFiles\Test_CAN.dir\Test_CAN_Test_CAN.ino.cpp.s

# Object files for target Test_CAN
Test_CAN_OBJECTS = \
"CMakeFiles/Test_CAN.dir/Test_CAN_Test_CAN.ino.cpp.obj"

# External object files for target Test_CAN
Test_CAN_EXTERNAL_OBJECTS =

Test_CAN.elf: CMakeFiles/Test_CAN.dir/Test_CAN_Test_CAN.ino.cpp.obj
Test_CAN.elf: CMakeFiles/Test_CAN.dir/build.make
Test_CAN.elf: libmega_mcp2515.a
Test_CAN.elf: libmega_global.a
Test_CAN.elf: libmega_mcp2515_defs.a
Test_CAN.elf: libmega_mcp2515_defs.a
Test_CAN.elf: libmega_global.a
Test_CAN.elf: libmega_mcp2515.a
Test_CAN.elf: libmega_Canbus.a
Test_CAN.elf: libmega_defaults.a
Test_CAN.elf: libmega_CORE.a
Test_CAN.elf: CMakeFiles/Test_CAN.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\Users\cyril\CLionProjects\Test_CAN\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable Test_CAN.elf"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\Test_CAN.dir\link.txt --verbose=$(VERBOSE)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating EEP image"
	"C:\Program Files (x86)\Arduino\hardware\tools\avr\bin\avr-objcopy.exe" -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 C:/Users/cyril/CLionProjects/Test_CAN/cmake-build-debug/Test_CAN.elf C:/Users/cyril/CLionProjects/Test_CAN/cmake-build-debug/Test_CAN.eep
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating HEX image"
	"C:\Program Files (x86)\Arduino\hardware\tools\avr\bin\avr-objcopy.exe" -O ihex -R .eeprom C:/Users/cyril/CLionProjects/Test_CAN/cmake-build-debug/Test_CAN.elf C:/Users/cyril/CLionProjects/Test_CAN/cmake-build-debug/Test_CAN.hex
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Calculating image size"
	"C:\Program Files\JetBrains\CLion 2019.3.5\bin\cmake\win\bin\cmake.exe" -DFIRMWARE_IMAGE=C:/Users/cyril/CLionProjects/Test_CAN/cmake-build-debug/Test_CAN.elf -DMCU=atmega2560 -DEEPROM_IMAGE=C:/Users/cyril/CLionProjects/Test_CAN/cmake-build-debug/Test_CAN.eep -P C:/Users/cyril/CLionProjects/Test_CAN/cmake-build-debug/CMakeFiles/FirmwareSize.cmake

# Rule to build all files generated by this target.
CMakeFiles/Test_CAN.dir/build: Test_CAN.elf

.PHONY : CMakeFiles/Test_CAN.dir/build

CMakeFiles/Test_CAN.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\Test_CAN.dir\cmake_clean.cmake
.PHONY : CMakeFiles/Test_CAN.dir/clean

CMakeFiles/Test_CAN.dir/depend: Test_CAN_Test_CAN.ino.cpp
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\cyril\CLionProjects\Test_CAN C:\Users\cyril\CLionProjects\Test_CAN C:\Users\cyril\CLionProjects\Test_CAN\cmake-build-debug C:\Users\cyril\CLionProjects\Test_CAN\cmake-build-debug C:\Users\cyril\CLionProjects\Test_CAN\cmake-build-debug\CMakeFiles\Test_CAN.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Test_CAN.dir/depend
