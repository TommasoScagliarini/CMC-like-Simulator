cmake_minimum_required(VERSION 3.15)

project(SEAPluginRelativeD)

set(PLUGIN_TARGET SEA_Plugin_BlackBox_mCMC_impedence_ff)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(APPLE)
    set(CMAKE_OSX_ARCHITECTURES "arm64")
endif()

find_package(OpenSim REQUIRED PATHS "${OPENSIM_INSTALL_DIR}" NO_DEFAULT_PATH)

add_library(${PLUGIN_TARGET} SHARED
    SeriesElasticActuator.cpp
    Plugin_interface.cpp
)

target_link_libraries(${PLUGIN_TARGET} ${OpenSim_LIBRARIES})

if(MSVC)
    target_compile_options(${PLUGIN_TARGET} PUBLIC "/W3")
else()
    target_compile_options(${PLUGIN_TARGET} PUBLIC "-fno-char8_t")
endif()
