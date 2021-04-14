# Velocity Skinning code replicability


## Compilation

### Linux

#### External dependencies

* A C++ compiler, cmake, libglfw3

* Command lines to set-up an Ubuntu system from scratch

```shell
# Basic development tools (g++, make, etc)
sudo apt-get install build-essential

# CMake 
sudo apt-get install cmake 

# GLFW
sudo apt-get install libglfw3-dev
```

#### Compile and execute the code

__Ex. to compile the code in 01_fig_generic using the default Makefile (for standard system)__

* Open a command line in 01_fig_generic/ directory 

```shell
make
./pgm
```


__Ex. to compile the code in 01_fig_generic using CMake (should work on more various systems)__

* Open a command line in 01_fig_generic/ directory

```shell
# Generate the directory build/
mkdir build 

# Go to the build/ directory
cd build

# Run CMake
cmake ..
# A file Makefile should be generated

# Compile
make
# Make sure the compilation succeed, a file pgm should be created

# Go back to the root directory
cd ..

# Run the executable from the root directory
build/sample_code 
```

### Windows

Method to compile with Visual Studio 2019 and CMake

#### Generate the Visual Studio project

* Start CMake (cmake-gui)
  *  Fill "Where is the source code" with the full path to the directory 01_fig_generic/
  * Fill "Where to build the binaries" with the full path to 01_fig_generic/build
* Click **Configure**
* Once the configuration is done, click on **Generate**

_If successfull, a build/ directory is created and contains the file **pgm.sln**_




#### Setup Visual Studio

* Start Visual Studio and open the project file `pgm.sln`

**In the solution explorer**

* Right click on Solution 'projectName' (Solution 'sample_code' in the sample code) and select **Properties**
* Change the value of `Single startup project` to `pgm` (this sets Visual Studio to compile the current project instead of a generic "All_BUILD" empty project).

![](assets/02_pgm_build.jpg)

**In the top toolbar**

* Change the build type from 'Debug' to '**RelWithDebInfo**' (this allows to have optimal runtime performance, while preserving debug information).

![](assets/03_RelWithDebInfo.jpg)

* In **Local Windows Debugger**, select 'projectName Debug Properties'.
* In the General properties (selected by default) change the value of **Output Directory** in removing the directory 'build\RelWithDebInfo\' from the path (the end of the pathname)
  * _(As a result, the executable `pgm.exe` will be generated in the root directory of the project instead of a build\RelWithDebInfo\ subdirectory)._

![](assets/04_local_windows_debugger.jpg)

![](assets/05_output_directory.jpg)

* In the **Debugging** property change the value of **Working Directory** from $(ProjectDir) to **$(TargetDir)**

* The project should now be ready to be compiled and run.

## Usage

### Camera control and interaction

- left click + mouse drag: rotation
- right click + mouse drag: zoom
- CTRL + left click + mouse drag: pan
- CTRL + right click + mouse drag: translation forward/backward
- SHIFT + left click on a joint: joint selection (followed by mouse drag to articlate it)

### Generic images

The directory `01_fig_generic/` contains the standard C++ implementation used to generate most of the illustrative images didn't requiering GPU acceleration.

This video describes the use of this code to generate most of the figures (such as Fig.4, 5, 9, and 10), and results shown in the video.

![](assets/use_code_01_fig_generic.jpg)

[![](assets/use_code_01_fig_generic.jpg)](assets/use_code_01_fig_generic.mp4)