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

# Globally define LVGL custom memory allocator functions
# (Using add_compile_definitions to ensure they are available to all components)
add_compile_definitions(CONFIG_LV_MEM_CUSTOM_ALLOC=lvgl_psram_malloc)
add_compile_definitions(CONFIG_LV_MEM_CUSTOM_FREE=lvgl_psram_free)
add_compile_definitions(CONFIG_LV_MEM_CUSTOM_REALLOC=lvgl_psram_realloc)

# Ensure the header lv_psram_mem.h is reachable from all components (including managed ones)
idf_build_set_property(COMPILE_OPTIONS "-I${CMAKE_CURRENT_SOURCE_DIR}/main" APPEND)

# Create littlefs image from main/flash_data directory
# Note: partition name must match the partition table (partitions.csv)
littlefs_create_partition_image(storage main/flash_data FLASH_IN_PROJECT)

# Define directories for update files
set(UPDATE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/update")
set(VERSION_DIR "${UPDATE_DIR}/update_movision-hd1_v${PROJECT_VER}")

# Post-build step to copy binaries after compilation
add_custom_target(copy_update_files ALL
    DEPENDS gen_project_binary

    # Create directory if it doesn't exist
    COMMAND ${CMAKE_COMMAND} -E make_directory ${VERSION_DIR}
    
    # Message for visibility
    COMMAND ${CMAKE_COMMAND} -E echo "Copying update files to ${UPDATE_DIR}..."

    # Copy the main app bin to VERSION_DIR
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin ${VERSION_DIR}/movision.bin
    
    # Copy partition table, bootloader, and other binaries to VERSION_DIR
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/partition_table/partition-table.bin ${VERSION_DIR}/
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/bootloader/bootloader.bin ${VERSION_DIR}/
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/ota_data_initial.bin ${VERSION_DIR}/
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/storage.bin ${VERSION_DIR}/

    # Copy BLE Update Tools to VERSION_DIR
    COMMAND ${CMAKE_COMMAND} -E copy ${UPDATE_DIR}/ble_update.bat ${VERSION_DIR}/
    COMMAND ${CMAKE_COMMAND} -E copy ${UPDATE_DIR}/ble_ota_tool.py ${VERSION_DIR}/
    
    # Copy Web Update Tools for PC without Python
    COMMAND ${CMAKE_COMMAND} -E copy ${UPDATE_DIR}/update_tool.html ${VERSION_DIR}/
    COMMAND ${CMAKE_COMMAND} -E copy ${UPDATE_DIR}/esptool_web.js ${VERSION_DIR}/esptool_web.js

    # Generate readme.txt in VERSION_DIR
    COMMAND cmd /c echo Model: MOVISION-HD1 > ${VERSION_DIR}/readme.txt
    COMMAND cmd /c echo Version: ${PROJECT_VER} >> ${VERSION_DIR}/readme.txt
    COMMAND cmd /c echo Build Time: ${BUILD_VERSION} >> ${VERSION_DIR}/readme.txt
    COMMAND cmd /c echo. >> ${VERSION_DIR}/readme.txt
    COMMAND cmd /c echo [How to update] >> ${VERSION_DIR}/readme.txt
    COMMAND cmd /c echo 1. USB Update: Open update_tool.html in Chrome/Edge browser >> ${VERSION_DIR}/readme.txt
    COMMAND cmd /c echo 2. BLE Update: Run ble_update.bat (No cable needed) >> ${VERSION_DIR}/readme.txt
    COMMAND cmd /c echo. >> ${VERSION_DIR}/readme.txt
    COMMAND cmd /c echo [Bin File Download Addresses] >> ${VERSION_DIR}/readme.txt
    COMMAND cmd /c echo 0x000000      bootloader.bin >> ${VERSION_DIR}/readme.txt
    COMMAND cmd /c echo 0x008000      partition-table.bin >> ${VERSION_DIR}/readme.txt
    COMMAND cmd /c echo 0x00f000      ota_data_initial.bin >> ${VERSION_DIR}/readme.txt
    COMMAND cmd /c echo 0x020000      movision.bin >> ${VERSION_DIR}/readme.txt
    COMMAND cmd /c echo 0x3b2000      storage.bin >> ${VERSION_DIR}/readme.txt
)
