cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(watara-lcd C CXX ASM)

set(CMAKE_BUILD_TYPE "MinSizeRel")

set(OUTPUT_DIR "${CMAKE_SOURCE_DIR}/bin/")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIR}")

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

option(TFT "Enable TFT video output" OFF)
option(VGA "Enable VGA video output" ON)

add_executable(${PROJECT_NAME})

add_subdirectory(drivers/graphics)
IF (TFT)
    add_subdirectory(drivers/st7789)
    target_link_libraries(${PROJECT_NAME} PRIVATE graphics st7789)
    message(STATUS "TFT Build")
    target_compile_definitions(${PROJECT_NAME} PRIVATE TFT)
    SET(BUILD_NAME "${BUILD_NAME}-TFT")
ELSE ()
    add_subdirectory(drivers/vga-nextgen)
    target_link_libraries(${PROJECT_NAME} PRIVATE graphics vga-nextgen)
    target_compile_definitions(${PROJECT_NAME} PRIVATE VGA)
    message(STATUS "VGA Build")
    SET(BUILD_NAME "${BUILD_NAME}-VGA")
ENDIF ()

target_compile_definitions(${PROJECT_NAME} PRIVATE
        PICO_PRINTF_ALWAYS_INCLUDED=0
        PICO_PANIC_FUNCTION=

        VGA_BASE_PIN=6

        TFT_CS_PIN=6 TFT_RST_PIN=8 TFT_LED_PIN=9 TFT_DC_PIN=10 TFT_DATA_PIN=12 TFT_CLK_PIN=13
)

pico_set_printf_implementation(${PROJECT_NAME} none)

add_compile_options(-flto -fwhole-program -ffunction-sections -fdata-sections -O3) # size optimizations

target_sources(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/main.c
)

target_include_directories(${PROJECT_NAME} PUBLIC)

target_link_libraries(${PROJECT_NAME} PUBLIC
        pico_runtime
        pico_multicore
        -Wl,--wrap=atexit # size optimizations
)

pico_set_binary_type(${PROJECT_NAME} copy_to_ram)
set(PICO_COPY_TO_RAM 1)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "${PROJECT_NAME}${BUILD_NAME}")
target_link_options(${PROJECT_NAME} PRIVATE -Xlinker --print-memory-usage --data-sections --function-sections)

