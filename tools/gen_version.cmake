# Get current timestamp (YYMMDD)
string(TIMESTAMP NEW_VERSION "%y%m%d")
set(VERSION_CONTENT "#define FIRMWARE_VERSION \"${NEW_VERSION}\"\n")

# 1. Update main/version.h for C code (ALWAYS write to force recompile)
file(WRITE "${VERSION_H}" "${VERSION_CONTENT}")
message("Version header updated (forced recompile): ${NEW_VERSION}")

# 2. Update version.txt for CMake re-configuration (ONLY update if date changed)
set(VERSION_TXT "${CMAKE_CURRENT_SOURCE_DIR}/version.txt")
if(EXISTS "${VERSION_TXT}")
    file(READ "${VERSION_TXT}" OLD_TXT_CONTENT)
    string(STRIP "${OLD_TXT_CONTENT}" OLD_TXT_CONTENT)
endif()

if(NOT "${NEW_VERSION}" STREQUAL "${OLD_TXT_CONTENT}")
    file(WRITE "${VERSION_TXT}" "${NEW_VERSION}\n")
    message("Version file updated for next config: ${NEW_VERSION}")
endif()
