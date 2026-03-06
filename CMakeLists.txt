# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Define firmware version based on build timestamp (YYMMDD)
string(TIMESTAMP BUILD_VERSION "%y%m%d")
set(PROJECT_VER ${BUILD_VERSION})

# Define the project
project(movision)

# Create littlefs image from main/flash_data directory
# Note: partition name must match the partition table (partitions.csv)
littlefs_create_partition_image(storage main/flash_data FLASH_IN_PROJECT)

# Define directories for update files
set(UPDATE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/update")
set(FACTORY_DIR "${UPDATE_DIR}/factory")
set(CONSUMER_DIR "${UPDATE_DIR}/consumer")

# Post-build step to copy binaries after compilation
add_custom_target(copy_update_files ALL
    DEPENDS gen_project_binary

    # Create directories if they don't exist
    COMMAND ${CMAKE_COMMAND} -E make_directory ${FACTORY_DIR}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CONSUMER_DIR}
    
    # Message for visibility
    COMMAND ${CMAKE_COMMAND} -E echo "Copying update files to ${UPDATE_DIR}..."

    # Copy the main app bin to consumer folder (for OTA update)
    COMMAND ${CMAKE_COMMAND} -E copy 
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin 
        ${CONSUMER_DIR}/${PROJECT_NAME}_ota.bin

    # Copy the main app bin to factory folder (for initial flash using serial)
    COMMAND ${CMAKE_COMMAND} -E copy 
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin 
        ${FACTORY_DIR}/${PROJECT_NAME}_factory.bin
    
    # Copy partition table and bootloader as well for factory flash reference (optional but helpful)
    COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_BINARY_DIR}/partition_table/partition-table.bin
        ${FACTORY_DIR}/
    COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_BINARY_DIR}/bootloader/bootloader.bin
        ${FACTORY_DIR}/
)
