# Harmony Research Interface Library

Code for the research interface library of Harmony.

## Dependencies
```
sudo apt update
sudo apt install -y libwebsocketpp-dev libjsoncpp-dev libbson-dev
sudo apt install -y ros-${ROS_DISTRO}-rosbridge-suite
```

## Build instructions

This repository uses [CMake](https://cmake.org) to build. The original library uses [Meson](https://mesounbuild.com). For compatibility with other code, the library has been moved to CMake.

### Installing prerequisite software
Not much to install before use:
* `sudo apt install cmake qtcreator`

### Usage
* CMake searches for the `harmony_research` precompiled library and imports it to the project.
* Check the CMake output to verify if harmony_research was found.
* Several tools are available to run tests and check if the code is working properly.
