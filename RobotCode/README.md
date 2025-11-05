# RobotCode Project

A C++ project for robot arm kinematics, using the Orocos-KDL library for calculations.

## Dependencies

* **CMake** (version 3.15 or higher)
* **A C++17 compatible compiler** (e.g., g++, clang++)

The Orocos-KDL library is included locally in the `lib/` directory as a pre-compiled shared object (`.so`) and does not need to be installed system-wide.

## How to Build

The project uses an "out-of-source" build pattern, which keeps the main directory clean.

1.  **Create a build directory:**
    ```bash
    mkdir build
    ```

2.  **Configure the project:**
    (This command tells CMake to look for the source in the current directory (`.`) and configure the build in the `build/` directory.)
    ```bash
    cmake -S . -B build
    ```

3.  **Compile the project:**
    (This runs the actual compilation process.)
    ```bash
    cmake --build build
    ```

## How to Run

After a successful build, the executable will be located in the `build` directory.

```bash
./build/RobotCode
