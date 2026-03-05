cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME test_foc)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Encoder
add_library(encoder STATIC)
target_sources(encoder PRIVATE uFOC_lib/encoder/Src/encoder.c)
target_include_directories(encoder PUBLIC uFOC_lib/encoder/Inc)
target_link_libraries(encoder PUBLIC stm32cubemx)


# Driver
add_library(driver STATIC)
target_sources(driver PRIVATE uFOC_lib/driver/Src/driver.c)
target_include_directories(driver PUBLIC uFOC_lib/driver/Inc)
target_link_libraries(driver PUBLIC stm32cubemx)

# Debug utils
add_library(debug_utils STATIC)
target_sources(debug_utils PRIVATE uFOC_lib/debug_utils/Src/debug_utils.c)
target_include_directories(debug_utils PUBLIC uFOC_lib/debug_utils/Inc)
target_link_libraries(debug_utils PUBLIC stm32cubemx)
target_link_libraries(debug_utils PUBLIC stm32cubemx driver)


# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    encoder
    driver
    m
    
    debug_utils
)
target_link_options(${PROJECT_NAME} PRIVATE
  -u _printf_float
)
