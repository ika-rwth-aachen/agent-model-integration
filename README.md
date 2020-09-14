# Driver model for SET Level 4to5

## Build
Brief build instructions for compilation with `MinGW` (64Bit V8.1.0) on Windows.  
### Protobuf dependency
Download a protobuf source (tested with 3.8.0) and follow the cmake build instructions (create `build` folder in `cmake` subdirectory). In addition, do the following:
* Use the `MinGW` compiler (i.e. `cmake -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release ..`)
* Set `CMAKE_INSTALL_PREFIX` to a desired directory 
* After building protobuf, make sure that directory is the first directory where a protobuf installation may be found in your `PATH` variable.
  
### Build FMU
First create a `build` directory and enter it. Then execute:
```
cmake -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release ..
```  

After building, the `FMU` will be in the subfolder `lib/`. If a specific `FMU` output dir shall be used:
```
cmake -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DFMU_OUTDIR=<dir> ..
```  

Compile the library with:
```
mingw32-make.exe
```
Optional: `mingw32-make.exe -j4` for building on multiple cores (replace `4` with an arbitrary number).

### Remarks
**The described build instructions should be very similar when using other compilers!**  
