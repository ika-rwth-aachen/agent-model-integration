cmake_minimum_required(VERSION 3.5)
project(osi_wrapper)

# Includes
set(SIMDRIVER_DIR ../SimDriver/src)
set(INJECTION_DIR ../SimDriver/lib/Injection)

include_directories(
	#${SIMDRIVER_DIR} 
	../json/single_include/
	#${OSI_INCLUDE_DIR}
	#${PROTOBUF_INCLUDE_DIR}
	#src/
	)

# Define target library ikaDriver
add_library(${PROJECT_NAME} STATIC 
	src/IkaAgent.cpp 
	src/OsiConverter.cpp 
	src/Logger.cpp 
	src/VehicleModel.cpp
	src/PrimaryController.cpp
	)

# Build ikaDriver after protobuf and osi has been downloaded and installed
add_dependencies(${PROJECT_NAME} protobuf_download osi_submodule)

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

add_executable(${PROJECT_NAME}_ositrace
	#src/IkaAgent.cpp
	#src/VehicleModel.cpp
	#src/OsiConverter.cpp
	#src/Logger.cpp
	#src/PrimaryController.cpp
	test/osiTraceTest.cpp
	)
target_link_libraries(${PROJECT_NAME}_ositrace
	#PUBLIC static_osi_lib
	#PUBLIC static_protobuf_lib
	#PUBLIC agent_model
	PUBLIC osi_wrapper
	)