# --- Project settings ---
cmake_minimum_required(VERSION 3.15)
project(RobotCode2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# This setting is specific to neovim users
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- Define Executable ---
# Create an executable target named 'RobotCode'
# from the source files in the 'src' directory
add_executable(RobotCode
    src/main.cpp
    src/robotArm.cpp
    src/armTester.cpp
)

# --- Include Directories ---
# Target where to find header files
target_include_directories(RobotCode PRIVATE
    # Project headers 
    ${CMAKE_CURRENT_SOURCE_DIR}/include

    # Library headers
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/orocos-kdl/include
)

# --- Link Libraries ---
# Tell 'RobotCode' to link against the pre-compiled Orocos-KDL
# shared library (.so file) located in your 'lib' folder
target_link_libraries(RobotCode PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/orocos-kdl/lib/liborocos-kdl.so.1.5.2
)

# Optional: Set RPATH to find the .so at runtime
# This tells the compiled executable to look in its own 'lib' directory
# (relative to the executable's location) when it runs.
# This is often needed when linking local shared libraries.
# We set it to $ORIGIN/../lib because the executable will be in 'build/'
# and the library is in 'lib/'.
set_target_properties(RobotCode PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib/orocos-kdl/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
)

