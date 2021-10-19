**README will be updated according to open source template**

# Driver model for SET Level 4to5
> CI pipeline results:  
> * [**Latest FMU** (Ubuntu 18.04 LTS)](https://gitlab.sl4to5.de/deliverables/model/traffic-agents/ika-driver/-/jobs/artifacts/master/raw/lib/ikaDriverAgent.fmu?job=buildFMU1804) 
> * [**Latest FMU** (Ubuntu 20.04 LTS)](https://gitlab.sl4to5.de/deliverables/model/traffic-agents/ika-driver/-/jobs/artifacts/master/raw/lib/ikaDriverAgent.fmu?job=buildFMU2004) 

## Build
Brief build instructions for compilation with `MSYS2 MinGW 64-bit` on Windows.  
### Protobuf dependency
Due to the usage of the CMake feature 'ExternalProject_Add()', there is no need to download and build protobuf from source source anymore. 
  
### Windows Build FMU
1. Start `MSYS2 MinGW 64-bit` shell
1. Create a `build` directory and enter it:
    ```
    mkdir build && cd build
    ```  

2. Execute CMake:
    ```
    cmake -G "MSYS Makefiles" -DCMAKE_BUILD_TYPE=Release ..
    ```  

    Please note: The default build directory for the `FMU` is the subfolder `lib/`. If a specific `FMU` output dir shall be used, set the variable `FMU_OUTDIR`, e.g.
    ```
    cmake -G "MSYS Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_TYPE=Release -DFMU_OUTDIR=<dir> ..
    ```  

3. Compile the library:
    ```
    make.exe
    ```
    Optional: `make.exe -j4` for building on multiple cores (replace `4` with an arbitrary number).

### Linux Build FMU
1. Create a `build` directory and enter it:
    ```
    mkdir build && cd build
    ```  

2. Execute CMake:
    ```
    cmake -DCMAKE_BUILD_TYPE=Release ..
    ```  

    Please note: The default build directory for the `FMU` is the subfolder `lib/`. If a specific `FMU` output dir shall be used, set the variable `FMU_OUTDIR`, e.g.
    ```
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_TYPE=Release -DFMU_OUTDIR=<dir> ..
    ```  

3. Compile the library:
    ```
    make
    ```
    Optional: `make -j4` for building on multiple cores (replace `4` with an arbitrary number).
