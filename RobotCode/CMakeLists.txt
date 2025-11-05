# --- Project settings ---
cmake_minimum_required(VERSION 3.15)
project(RobotCodeCMake)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# This setting is specific to neovim users
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- Define Executable ---
# Create an executable target named 'RobotCode'
# from the source files in the 'src' directory
add_executable(RobotCode
    src/armTester.cpp
    src/driverStation.cpp
    src/main.cpp
    src/robotActuation.cpp
    src/robotArm.cpp
    src/robotControl.cpp
    src/robotSerial.cpp
    src/robotState.cpp
    src/util.cpp
)

# --- Include Directories ---
# Target where to find header files
target_include_directories(RobotCode PRIVATE
    # Project headers
    ${CMAKE_CURRENT_SOURCE_DIR}/include

    # Library headers for Orocos-KDL
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/orocos-kdl/include

    # Library headers for Asio (header-only)
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/asio-1.30-2/include
)

# --- Link Libraries ---
# Tell 'RobotCode' to link against the pre-compiled Orocos-KDL
# shared library (.so file) located in your 'lib' folder
target_link_libraries(RobotCode PRIVATE
    # We only need to link Orocos-KDL. Asio is header-only.
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/orocos-kdl/lib/liborocos-kdl.so.1.5.2
)

# Optional: Set RPATH to find the .so at runtime
set_target_properties(RobotCode PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib/orocos-kdl/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
)
