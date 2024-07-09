# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)
#message([WARNING] "Hello")

#set(CMAKE_C_COMPILER "c:\Users\Raven\.pico-sdk\ninja\v1.11.1")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
   set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(PICO_SDK_PATH ${USERHOME}/.pico-sdk/sdk/1.5.1)
set(PICO_TOOLCHAIN_PATH ${USERHOME}/.pico-sdk/toolchain/13_2_Rel1)
if(WIN32)
    set(pico-sdk-tools_DIR ${USERHOME}/.pico-sdk/tools/1.5.1)
    include(${pico-sdk-tools_DIR}/pico-sdk-tools-config.cmake)
    include(${pico-sdk-tools_DIR}/pico-sdk-tools-config-version.cmake)
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(Hexapod CXX C ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory("external")
# Add executable. Default name is the project name, version 0.1

set(${PROJECT_NAME}_SRCS
    src/Animation.cpp
    src/Calculate.cpp
    src/Hexapod.cpp
    src/Move.cpp
)

set(${PROJECT_NAME}_HDRS
    includes/Animation.h
    includes/Calculate.h
    includes/Hexapod.h
    includes/Move.h
    includes/DataPacket.h
)

add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS} ${${PROJECT_NAME}_HDRS})

pico_set_program_name(Hexapod "Hexapod")
pico_set_program_version(Hexapod "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Hexapod 0)
pico_enable_stdio_usb(Hexapod 1)

# Add the standard library to the build
target_link_libraries(${PROJECT_NAME}
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
  ${CMAKE_CURRENT_LIST_DIR}/includes
  ${CMAKE_CURRENT_LIST_DIR}/external/pimoroni-pico
  ${PICO_SDK_PATH}/include # add this to ensure SDK headers are included
)

# Add any user requested libraries
target_link_libraries(${PROJECT_NAME} 
        hardware_i2c
        pico_multicore
        servo2040
        button
        analogmux
        analog
        )

pico_add_extra_outputs(Hexapod)

