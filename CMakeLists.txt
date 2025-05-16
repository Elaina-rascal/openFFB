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
set(CMAKE_CXX_STANDARD 20)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

add_compile_options(-D_GLIBCXX_USE_CXX11_ABI=0)
add_compile_options(-frtti)
remove_definitions(-fno-rtti)

# file(GLOB MY_SRC 
#     "${CMAKE_CURRENT_SOURCE_DIR}/FFBoard/**/*.c"
#     "${CMAKE_CURRENT_SOURCE_DIR}/FFBoard/**/*.cpp"
# )
file(GLOB_RECURSE MY_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/FFBoard/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/FFBoard/*.cpp"
)
file(GLOB_RECURSE ALL_PATHS
    LIST_DIRECTORIES true
    "${CMAKE_CURRENT_SOURCE_DIR}/FFBoard/*"
)

# 过滤出所有是目录的路径
set(MY_INC "")

foreach(path ${ALL_PATHS})
    if(IS_DIRECTORY "${path}")
        list(APPEND MY_INC "${path}")
    endif()
endforeach()
# Set the project name
set(CMAKE_PROJECT_NAME OpenFFBoard_F411RE)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

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
    ${MY_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/newlib_lock_glue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/cpp_target_config.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user def ined include paths
    ${MY_INC}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
