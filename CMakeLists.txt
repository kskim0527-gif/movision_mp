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
set(VERSION_DIR "${UPDATE_DIR}/update_movision-hd1_v${PROJECT_VER}")

# Post-build step to copy binaries after compilation
add_custom_target(copy_update_files ALL
    DEPENDS gen_project_binary

    # Create directories if they don't exist
    COMMAND ${CMAKE_COMMAND} -E make_directory ${FACTORY_DIR}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CONSUMER_DIR}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${VERSION_DIR}
    
    # Message for visibility
    COMMAND ${CMAKE_COMMAND} -E echo "Copying update files to ${UPDATE_DIR}..."

    # Copy the main app bin for various targets
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin ${CONSUMER_DIR}/${PROJECT_NAME}_ota.bin
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin ${FACTORY_DIR}/${PROJECT_NAME}_factory.bin
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin ${VERSION_DIR}/update.bin
    
    # Copy partition table, bootloader, and other binaries to VERSION_DIR
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/partition_table/partition-table.bin ${VERSION_DIR}/
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/bootloader/bootloader.bin ${VERSION_DIR}/
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/ota_data_initial.bin ${VERSION_DIR}/
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/storage.bin ${VERSION_DIR}/

    # Copy templates to VERSION_DIR
    COMMAND ${CMAKE_COMMAND} -E copy ${UPDATE_DIR}/usb_update.bat.template ${VERSION_DIR}/usb_update.bat
    
    # Copy Web Update Tools for PC without Python
    COMMAND ${CMAKE_COMMAND} -E copy ${UPDATE_DIR}/update_tool.html ${VERSION_DIR}/
    COMMAND ${CMAKE_COMMAND} -E copy ${UPDATE_DIR}/esptool_web.js ${VERSION_DIR}/esptool_web.js

    # Generate readme.txt in VERSION_DIR
    COMMAND cmd /c echo Model: MOVISION-HD1 > ${VERSION_DIR}/readme.txt
    COMMAND cmd /c echo Version: ${PROJECT_VER} >> ${VERSION_DIR}/readme.txt
    COMMAND cmd /c echo Build Time: ${BUILD_VERSION} >> ${VERSION_DIR}/readme.txt
    COMMAND cmd /c echo. >> ${VERSION_DIR}/readme.txt
    COMMAND cmd /c echo [How to update] >> ${VERSION_DIR}/readme.txt
    COMMAND cmd /c echo 1. If Python is installed: Run usb_update.bat >> ${VERSION_DIR}/readme.txt
    COMMAND cmd /c echo 2. If NO Python: Open update_tool.html in Chrome/Edge browser >> ${VERSION_DIR}/readme.txt
)
