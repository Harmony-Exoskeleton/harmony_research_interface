# Harmony Research Interface Library

Code for the research interface library of Harmony.

## Build instructions

This repository uses [Meson](https://mesonbuild.com/) to build. It's recommended that you use the same versions of Python, Meson and Ninja specified in the [GitHub actions file](.github/workflows/build-test-format-check.yml).

### Installing prerequisite software
Let's install pip and ninja for future steps
* `sudo apt install build-essential python3-pip ninja-build`

### Installing Meson
Don't install Meson using `apt` or you will get an old version. Instead install it with `pip3`:
* `pip3 install --user meson` will install Meson in `~/.local/bin`
* `export PATH=~/.local/bin:$PATH` will add `~/.local/bin` to your `PATH`

### Building
The first time you need to build, you should run `meson setup builddir` from the project root. This will create a new `builddir` directory, download dependencies, and set up configuration.

Once the build directory is set up, you can build by running `meson compile -C builddir` from the project root.

### Dependencies
The project's dependencies are specified with Meson wrap files in the [subprojects](subprojects) directory. Do not install these dependencies manually but instead let Meson download and use the versions specified.

To add a new dependency, use [wraptool](https://mesonbuild.com/Using-wraptool.html).

### Testing
The Github repository is configured to build and run tests on each commit. To ensure that tests are passing before checking in, run `meson test -C builddir`

### Formatting
This repository uses `clang-format` to ensure consistent formatting of the codebase. To format the codebase, run `ninja -C builddir clang-format` **Note: Running this will modify source files. It's recommended that you stage or commit your changes locally before running clang-format.**

If you would like to check if code needs to be formatted without modifying files, use `ninja -C builddir clang-format-check`
