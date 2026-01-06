


<p align="center">
  <img src="https://github.com/user-attachments/assets/221ffc3f-f48d-4322-8e4f-9a0aebdad81c" alt="External_DRFL_1" width="250"/>
</p>
<p align="center">
  Copyright © 2025 Doosan Robotics Inc
</p>

# Doosan Robotics Framework Library


This document outlines the procedure for building the DRFL on Windows and Linux platforms. For detailed instructions, please refer to the official manual linked below:

[Official Doosan Robotics API Manual](https://doosanrobotics.github.io/doosan-robotics-api-manual/GL013301/)

## DRCF Version Compatibility

This code uses a preprocessor macro (`DRCF_VERSION`) to ensure compatibility with different versions of the Doosan Robot Controller Framework (DRCF). 

**To specify your DRCF version:**

* **For DRCF v2:** Set `DRCF_VERSION` to `2`.
* **For DRCF v3:** Set `DRCF_VERSION` to `3`.

**Example:**

```c++
#ifndef DRCF_VERSION
    #define DRCF_VERSION 3 // Set to 3 for DRCF v3
#endif
```

## System Requirements

Please ensure your environment meets the following conditions:

- **Library Composition**: Refer to the structure of the library via this [link](https://doosanrobotics.github.io/doosan-robotics-api-manual/GL013301/introduction/architecture_library/composition.html).
- **Recommended Specifications**: Review the recommended operational specifications [here](https://doosanrobotics.github.io/doosan-robotics-api-manual/GL013301/introduction/environment/system_requirements.html).


## Build Guidelines

### Windows (64-bit)

To build the Windows example, utilize the Visual Studio 2017 solution file provided at the link below:

[Windows Example Solution](https://github.com/DoosanRobotics/API-DRFL/blob/GL013301/example/Windows/windows_example/windows_example.sln)


### Linux (64-bit)

#### Automated Build Scripts

For easier building and cleaning, you can use the provided automated scripts:

**Build Script Usage:**
```bash
./API_DRFL_BUILD.sh
```
- Automatically detects Ubuntu version and architecture
- Scans all .cpp files in `example/Linux_64` directory
- Allows you to select which file to build (1, 2, 3...)
- Handles library linking automatically
- Creates executables in `out/` directory
- Optionally runs the built executable

**Clean Script Usage:**
```bash
./API_DRFL_CLEAN.sh
```
- Removes all .o object files
- Removes the `out/` directory and all executables
- Asks for confirmation before cleaning