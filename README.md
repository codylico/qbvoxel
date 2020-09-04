# qbvoxel
Parser/generator for Qubicle Binary file format

## Goals
- Full support for the Qubicle Binary (QB) File Specification.
- Simple and orthogonal function interface
- Language feature usage kept to a minimum

## Build
This project uses CMake
([https://cmake.org/download/](https://cmake.org/download/)).

To use CMake with this project, first create a directory to hold the compiled
code. Then run CMake in the directory with a path to the source code. Here
are sample commands for UNIX:
```
mkdir build
cd build
cmake ../qbvoxel
```

Running CMake should produce either an IDE project or a build script
(i.e. Makefile, ninja file, ...). For Makefiles, the following command
should build the project:
```
make
```
For IDE projects, the IDE must be installed and ready to use. Open the
project within the IDE.
