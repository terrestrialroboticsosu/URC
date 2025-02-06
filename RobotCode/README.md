# Rpi Setup and Compile

If you want to build this code on your own raspberry pi or linux machine for testing be sure to install the following dependencies first.

```
cd RobotCode
sh install.sh
```

Then to compile and execute the code open the PiCode directory and run the following commands:

```
meson setup build
meson compile -C build
cd build
./urcbot
```

If you use clangd as your LSP the compile_commands.json will be output to the build folder after compilation

## Adding new source files

When contributing new source files make sure to place all headers into the include folder and all C++ files into the src folder.

Then to ensure that the files are properly compiled add all new C++ sources to the ```proj_sources``` field in the meson.build file.

