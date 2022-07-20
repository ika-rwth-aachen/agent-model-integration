cmake_minimum_required(VERSION 3.5)
project(osi_wrapper)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build agent model library and set source dir
add_subdirectory(lib/SimDriver)
set(SIMDRIVER_SRC lib/SimDriver/src)

if(WIN32)
	set(PROTOBUF_CMAKE_GENERATOR "MSYS Makefiles")
	set(PROTOC_EXE "protoc.exe")
	set(OSI_CMAKE_GENERATOR "MSYS Makefiles")
elseif(UNIX)
	set(PROTOBUF_CMAKE_GENERATOR "Unix Makefiles")
	set(PROTOC_EXE "protoc")
	set(OSI_CMAKE_GENERATOR "Unix Makefiles")
endif()

# Install protobuf and OSI 
include(ExternalProject)
if(WIN32)
	set(PROTOBUF_CMAKE_GENERATOR "MSYS Makefiles")
	set(PROTOC_EXE "protoc.exe")
	set(OSI_CMAKE_GENERATOR "MSYS Makefiles")
elseif(UNIX)
	set(PROTOBUF_CMAKE_GENERATOR "Unix Makefiles")
	set(PROTOC_EXE "protoc")
	set(OSI_CMAKE_GENERATOR "Unix Makefiles")
endif()

# Download protobuf and install with position independent code 
# (pic) so that other Osi and ikaDriver can link against
ExternalProject_Add (protobuf_download
	PREFIX            	protobuf_download
	GIT_REPOSITORY    	https://github.com/protocolbuffers/protobuf.git
	GIT_TAG           	v3.11.4
	UPDATE_COMMAND    	""
	INSTALL_COMMAND   	make install
	BUILD_COMMAND     	make -j10
	BUILD_ALWAYS      	OFF
	INSTALL_DIR       	${CMAKE_CURRENT_BINARY_DIR}/ext/protobuf_download
	SOURCE_SUBDIR	  	cmake
	CONFIGURE_COMMAND 	${CMAKE_COMMAND} -G ${PROTOBUF_CMAKE_GENERATOR}
							-DCMAKE_BUILD_TYPE=Release 
							-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> 
							-DCMAKE_CXX_FLAGS="-fPIC" 
							-Dprotobuf_MSVC_STATIC_RUNTIME=OFF 
							-Dprotobuf_BUILD_TESTS=OFF 
							../protobuf_download/cmake)

# Download Osi and compile such that it links against the pic-build protobuf
ExternalProject_Add (osi_submodule
	DEPENDS             protobuf_download
	#PREFIX             osi_download
	#GIT_REPOSITORY     https://gitlab.setlevel.de/deliverables/architecture/open-simulation-interface.git
	#GIT_TAG            sl45/v3.2.2
	SOURCE_DIR 			${CMAKE_CURRENT_LIST_DIR}/lib/open-simulation-interface
	UPDATE_COMMAND      ""
	INSTALL_COMMAND     make install
	BUILD_COMMAND      	make -j10
	BUILD_ALWAYS        OFF
	INSTALL_DIR         ${CMAKE_CURRENT_BINARY_DIR}/ext/osi_submodule
	SOURCE_SUBDIR       cmake
	CONFIGURE_COMMAND 	${CMAKE_COMMAND} -G ${OSI_CMAKE_GENERATOR}
							-DCMAKE_BUILD_TYPE=Release 
							-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> 
							-DPROTOBUF_INCLUDE_DIR=${CMAKE_CURRENT_BINARY_DIR}/ext/protobuf_download/include 
							-DPROTOBUF_PROTOC_EXECUTABLE=${CMAKE_CURRENT_BINARY_DIR}/ext/protobuf_download/bin/${PROTOC_EXE} 
							-DPROTOBUF_LIBRARY=${CMAKE_CURRENT_BINARY_DIR}/ext/protobuf_download/lib/libprotobuf.a
							${CMAKE_CURRENT_LIST_DIR}/lib/open-simulation-interface)

# Get Osi installation directory
ExternalProject_Get_Property(osi_submodule INSTALL_DIR)
set(OSI ${INSTALL_DIR})
message("Osi installation directory :" ${OSI})

# Get Protobuf installation directory
ExternalProject_Get_Property(protobuf_download INSTALL_DIR)
set(PROTOBUF ${INSTALL_DIR})
message("Protobuf installation directory " ${PROTOBUF})

# Json library for debug files
set(JSON_LIB lib/json/include)
# Debug output directory
add_definitions(-DDEBUG_OUTDIR="${CMAKE_SOURCE_DIR}/debug")

# Setup osi wrapper library
add_library(${PROJECT_NAME} STATIC 
	src/IkaAgent.cpp 
	src/OsiConverter.cpp 
	src/Logger.cpp 
	src/VehicleModel.cpp
	src/PrimaryController.cpp
    )
set_target_properties(${PROJECT_NAME} PROPERTIES
    POSITION_INDEPENDENT_CODE ON)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${SIMDRIVER_SRC}
    ${OSI}/include/osi3
    ${PROTOBUF}/include
    ${JSON_LIB}
    src)

# Build ikaDriver after protobuf and osi has been downloaded and installed
add_dependencies(${PROJECT_NAME} protobuf_download osi_submodule)
# Define libraries which osi wrapper can link against
add_library(static_protobuf_lib STATIC IMPORTED)
add_dependencies(static_protobuf_lib protobuf_download)
set_target_properties(static_protobuf_lib PROPERTIES 
    IMPORTED_LOCATION "${PROTOBUF}/lib/libprotobuf.a")
add_library(static_osi_lib STATIC IMPORTED)
add_dependencies(static_osi_lib osi_submodule)
set_target_properties(static_osi_lib PROPERTIES 
	IMPORTED_LOCATION "${OSI}/lib/osi3/libopen_simulation_interface_pic.a")

if(WIN32)
	target_link_libraries(${PROJECT_NAME}
		PUBLIC static_osi_lib
		PUBLIC static_protobuf_lib
		PUBLIC agent_model -static-libgcc -static-libstdc++ -static
		)	
elseif(UNIX)
	target_link_libraries(${PROJECT_NAME}
		PUBLIC static_osi_lib
		PUBLIC static_protobuf_lib
		PUBLIC agent_model
		)
endif()

# App to process OSI traces
add_executable(${PROJECT_NAME}_ositrace
	test/osiTraceTest.cpp
	)
target_link_libraries(${PROJECT_NAME}_ositrace
	PUBLIC osi_wrapper
	)