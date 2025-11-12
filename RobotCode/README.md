# RobotCode 

This code is what controls the movement, arm, and communication on behalf of the robot.

## Dependencies

* **CMake** (version 3.15 or higher)
* **A C++17 compatible compiler** (e.g., g++, clang++)

The Orocos-KDL and Asio libraries are included locally in the `lib/` directory. Orocos-KDL is available as a pre-compiled shared object (`.so`) while Asio is a header-only library.

## How to Build

1.  **Create a build directory:**
    ```bash
    mkdir build
    ```

2.  **Build the project:**
    ```bash
    cmake -S . -B build
    ```

3.  **Compile the project:**
    ```bash
    cmake --build build
    ```

## How to Run

After a successful build, the executable will be located in the `build` directory.

```bash
./build/RobotCode
```
