# --- Project settings ---
cmake_minimum_required(VERSION 3.15)
project(RobotCode)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- Find External Dependencies ---
# KDL requires Eigen. This finds it on the system (Ubuntu/Fedora usually install it to /usr/include/eigen3)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# --- Define Executable ---
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
target_include_directories(RobotCode PRIVATE
    # Project headers
    ${CMAKE_CURRENT_SOURCE_DIR}/include

    # Library headers for Orocos-KDL
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/orocos-kdl/include

    # Library headers for Asio
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/asio-1.30-2/include
)

# --- Link Libraries ---
target_link_libraries(RobotCode PRIVATE
    # Link the vendored KDL library
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/orocos-kdl/lib/liborocos-kdl.so.1.5.2

    # Link Eigen (Header only, but this sets up the include paths correctly)
    Eigen3::Eigen
)

# Optional: Set RPATH to find the .so at runtime
set_target_properties(RobotCode PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib/orocos-kdl/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
)
