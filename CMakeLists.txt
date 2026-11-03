cmake_minimum_required(VERSION 3.20)

# Editor project only. Build/flash the firmware with AURIX Development Studio.
project(Qcar_IntelliSense LANGUAGES C)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB_RECURSE QCAR_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/code/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/user/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/libraries/*.c")

# Keep the same source exclusions as the ADS project.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/.cproject" QCAR_ADS_PROJECT)
string(REGEX MATCH "<entry excluding=\"([^\"]*)\"" QCAR_SOURCE_ENTRY "${QCAR_ADS_PROJECT}")
string(REPLACE "|" ";" QCAR_EXCLUSIONS "${CMAKE_MATCH_1}")
foreach(QCAR_SOURCE IN LISTS QCAR_SOURCES)
    file(RELATIVE_PATH QCAR_RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${QCAR_SOURCE}")
    foreach(QCAR_EXCLUSION IN LISTS QCAR_EXCLUSIONS)
        string(FIND "${QCAR_RELATIVE}" "${QCAR_EXCLUSION}/" QCAR_PREFIX)
        if(QCAR_PREFIX EQUAL 0)
            list(REMOVE_ITEM QCAR_SOURCES "${QCAR_SOURCE}")
            break()
        endif()
    endforeach()
endforeach()

# Read the include search order directly from the existing ADS configuration.
string(REGEX MATCHALL "\\$\\{workspace_loc:/\\$\\{ProjName\\}/[^}]*\\}"
    QCAR_ADS_INCLUDES "${QCAR_ADS_PROJECT}")
set(QCAR_INCLUDE_DIRS)
foreach(QCAR_INCLUDE IN LISTS QCAR_ADS_INCLUDES)
    string(REGEX REPLACE "^.*ProjName\\}/([^}]*)\\}.*$" "\\1" QCAR_INCLUDE "${QCAR_INCLUDE}")
    if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${QCAR_INCLUDE}")
        list(APPEND QCAR_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/${QCAR_INCLUDE}")
    endif()
endforeach()
if(NOT QCAR_INCLUDE_DIRS)
    message(FATAL_ERROR "Cannot read include paths from .cproject")
endif()

add_library(qcar_intellisense OBJECT EXCLUDE_FROM_ALL ${QCAR_SOURCES})
target_include_directories(qcar_intellisense PRIVATE ${QCAR_INCLUDE_DIRS})
set_target_properties(qcar_intellisense PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED YES)
target_compile_definitions(qcar_intellisense PRIVATE
    __TASKING__=1 __CPU__=tc26xb QCAR_VSCODE_ANALYSIS=1)
target_compile_options(qcar_intellisense PRIVATE
    -m32 -include "${CMAKE_CURRENT_SOURCE_DIR}/.vscode/tasking_intellisense.h")

add_custom_target(qcar_editor_info ALL
    COMMAND ${CMAKE_COMMAND} -E echo
    "Editor configuration ready. Use AURIX Development Studio to build firmware.")
