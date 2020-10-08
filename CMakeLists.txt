cmake_minimum_required(VERSION 3.5)
project(ikaDriverAgent)
# Set a default build type if none was specified
set(default_build_type "Release")
if(EXISTS "${CMAKE_SOURCE_DIR}/../.git")
  set(default_build_type "Debug")
endif()

set(FMU_OUTDIR "${CMAKE_SOURCE_DIR}/lib" CACHE STRING "Fmu output directory")

add_subdirectory(lib/open-simulation-interface)
add_subdirectory(lib/SimDriver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(LINK_WITH_SHARED_OSI ON CACHE BOOL "Link FMU with shared OSI library instead of statically linking")
set(PUBLIC_LOGGING OFF CACHE BOOL "Enable logging via FMI logger")
set(PRIVATE_LOGGING OFF CACHE BOOL "Enable private logging to file")
if(WIN32)
	set(PRIVATE_LOG_PATH "C:/TEMP/${PROJECT_NAME}.log" CACHE FILEPATH "Path to write private log file to")
else()
	set(PRIVATE_LOG_PATH "/tmp/${PROJECT_NAME}.log" CACHE FILEPATH "Path to write private log file to")
endif()
set(VERBOSE_FMI_LOGGING OFF CACHE BOOL "Enable detailed FMI function logging")
set(DEBUG_BREAKS OFF CACHE BOOL "Enable debugger traps for debug builds of FMU")

string(TIMESTAMP FMUTIMESTAMP UTC)
string(MD5 FMUGUID modelDescription.in.xml)
configure_file(modelDescription.xml modelDescription.xml @ONLY)

find_package(Protobuf REQUIRED)
set(SIMDRIVER_SRC lib/SimDriver/src)
set(INJECTION_DIR lib/SimDriver/lib/Injection)
#message("osi dir " ${CMAKE_CURRENT_BINARY_DIR}/lib/open-simulation-interface)
set(SIMDRIVER_LIB ${CMAKE_CURRENT_BINARY_DIR}/lib/SimDriver/src)
include_directories(
	include
	include/FMI 
	#${THIRDPARTY_INCLUDES} 
	#${THIRDPARTY_INCLUDES}/osi3 
	${SIMDRIVER_SRC} 
	${Protobuf_INCLUDE_DIRS}
	#${INJECTION_INC}/include/injection
)
#link_directories(${THIRDPARTY_LIBRARIES} ${Protobuf_LIBRARIES} ${SIMDRIVER_LIB} ${CMAKE_CURRENT_BINARY_DIR}/lib/open-simulation-interface)
#link_directories(${THIRDPARTY_LIBRARIES})
#link_directories(${Protobuf_LIBRARIES})
add_library(${PROJECT_NAME} SHARED 
			src/OSMPTrafficAgent.cpp 
			src/IkaAgent.cpp 
			src/VehicleModel.cpp
			src/PrimaryController.cpp
			#${SIMDRIVER_SRC}/AgentModel.cpp
			#${SIMDRIVER_SRC}/model_collection.cpp
			#${SIMDRIVER_SRC}/AgentModelInjection.cpp
			)
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")

target_compile_definitions(${PROJECT_NAME} PRIVATE "FMU_SHARED_OBJECT")
target_compile_definitions(${PROJECT_NAME} PRIVATE "FMU_GUID=\"${FMUGUID}\"")
if(WIN32)
    target_link_libraries(${PROJECT_NAME}
                  PUBLIC open_simulation_interface_static
                  #PUBLIC protobuf
                  PUBLIC agent_model)
else()
  target_link_libraries(${PROJECT_NAME}
                  PUBLIC open_simulation_interface
		  #PUBLIC protobuf
		  PUBLIC agent_model)
endif()
#if(LINK_WITH_SHARED_OSI)
	#target_link_libraries(${PROJECT_NAME} open_simulation_interface -lprotobuf agent_model)
	

#else()
	#target_link_libraries(${PROJECT_NAME} open_simulation_interface_pic agent_model injection)
#endif()
if(PRIVATE_LOGGING)
	file(TO_NATIVE_PATH ${PRIVATE_LOG_PATH} PRIVATE_LOG_PATH_NATIVE)
	string(REPLACE "\\" "\\\\" PRIVATE_LOG_PATH_ESCAPED ${PRIVATE_LOG_PATH_NATIVE})
	target_compile_definitions(${PROJECT_NAME} PRIVATE
		"PRIVATE_LOG_PATH=\"${PRIVATE_LOG_PATH_ESCAPED}\"")
endif()
target_compile_definitions(${PROJECT_NAME} PRIVATE
	$<$<BOOL:${PUBLIC_LOGGING}>:PUBLIC_LOGGING>
	$<$<BOOL:${VERBOSE_FMI_LOGGING}>:VERBOSE_FMI_LOGGING>
	$<$<BOOL:${DEBUG_BREAKS}>:DEBUG_BREAKS>)

if(WIN32)
	if(CMAKE_SIZEOF_VOID_P EQUAL 8)
		set(FMI_BINARIES_PLATFORM "win64")
	else()
		set(FMI_BINARIES_PLATFORM "win32")
	endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	if(CMAKE_SIZEOF_VOID_P EQUAL 8)
		set(FMI_BINARIES_PLATFORM "linux64")
	else()
		set(FMI_BINARIES_PLATFORM "linux32")
	endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	if(CMAKE_SIZEOF_VOID_P EQUAL 8)
		set(FMI_BINARIES_PLATFORM "darwin64")
	else()
		set(FMI_BINARIES_PLATFORM "darwin32")
	endif()
endif()

get_filename_component(Protobuf_REAL_LIBRARY_FILENAME "${Protobuf_LIBRARY}" REALPATH)

add_custom_command(OUTPUT ${FMU_OUTDIR}/${PROJECT_NAME}.fmu
                   DEPENDS ${PROJECT_NAME}
                           modelDescription.xml
                   COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu"
                   COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources"
                   COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}"
                   COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/modelDescription.xml" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu"
                   COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}"
                   #COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:open_simulation_interface> "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}"
                   #COMMAND ${CMAKE_COMMAND} -E copy "${Protobuf_REAL_LIBRARY_FILENAME}" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}"
				   #COMMAND ${CMAKE_COMMAND} -E chdir "${CMAKE_CURRENT_BINARY_DIR}/buildfmu" ${CMAKE_COMMAND} -E tar "cfv" "${CMAKE_CURRENT_SOURCE_DIR}/lib/${PROJECT_NAME}.fmu" --format=zip "modelDescription.xml" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}")
				   COMMAND ${CMAKE_COMMAND} -E chdir "${CMAKE_CURRENT_BINARY_DIR}/buildfmu" ${CMAKE_COMMAND} -E tar "cfv" "${FMU_OUTDIR}/${PROJECT_NAME}.fmu" --format=zip "modelDescription.xml" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}")

add_custom_target(${PROJECT_NAME}_fmu
                  ALL
				  DEPENDS ${FMU_OUTDIR}/${PROJECT_NAME}.fmu)
				  

#add_custom_command(TARGET ${PROJECT_NAME}
	#POST_BUILD
	#COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu"
	#COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources"
	#COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}"
	#COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/modelDescription.xml" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu"
	#COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/src/IkaAgent.cpp" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources/"
	##COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/../SimDriver/src/AgentModel.cpp" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources/"
	#COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/include/IkaAgent.h" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources/"
	#COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}.cpp" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources/"
	#COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}.h" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources/"
	#COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> $<$<PLATFORM_ID:Windows>:$<$<CONFIG:Debug>:$<TARGET_PDB_FILE:${PROJECT_NAME}>>> "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}"
	#COMMAND ${CMAKE_COMMAND} -E chdir "${CMAKE_CURRENT_BINARY_DIR}/buildfmu" ${CMAKE_COMMAND} -E tar "cfv" "../${PROJECT_NAME}.fmu" --format=zip "modelDescription.xml" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}")
