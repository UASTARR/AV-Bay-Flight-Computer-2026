cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Add sources to executable
set(I2C_Application_Src
    # Add user sources here
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/software_i2c/stm32_sw_i2c.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/software_i2c/dwt_stm32_delay.c
)

# Add include paths
set(I2C_Include_Dirs
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/software_i2c/stm32_sw_i2c.h
     ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/software_i2c/dwt_stm32_delay.h
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${I2C_Application_Src})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${I2C_Include_Dirs})