cmake_minimum_required(VERSION 3.14)

macro(default name)
    if(NOT DEFINED "${name}")
        set("${name}" "${ARGN}")
    endif()
endmacro()

default(FIX NO)

set(flag --output-replacements-xml)
set(args OUTPUT_VARIABLE output)
if(FIX)
    set(flag -i)
    set(args "")
endif()

file(
    GLOB_RECURSE
    clang_files
    Sts1CobcSw/*.cpp
    Sts1CobcSw/*.hpp
    Sts1CobcSw/*.ipp
    Tests/*.cpp
    Tests/*.hpp
    Tests/*.ipp
)

file(GLOB_RECURSE cmake_files cmake/*.cmake CMakeLists.txt)

set(badly_formatted "")
set(output "")
string(LENGTH "${CMAKE_SOURCE_DIR}/" path_prefix_length)

foreach(file IN LISTS clang_files)
    execute_process(
        COMMAND clang-format --style=file "${flag}" "${file}"
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        RESULT_VARIABLE result ${args}
    )
    if(NOT result EQUAL "0")
        message(FATAL_ERROR "'${file}': formatter returned with ${result}")
    endif()
    if(NOT FIX AND output MATCHES "\n<replacement offset")
        string(SUBSTRING "${file}" "${path_prefix_length}" -1 relative_file)
        list(APPEND badly_formatted "${relative_file}")
    endif()
    set(output "")
endforeach()

foreach(file IN LISTS cmake_files)
    execute_process(
        if (NOT FIX) 
            set (flag "") 
        endif ()
        COMMAND cmake-format --style=file "${flag}" "${file}"
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        RESULT_VARIABLE result ${args}
    )
    if(NOT result EQUAL "0")
        message(FATAL_ERROR "'${file}': formatter returned with ${result}")
    endif()
    if(NOT FIX AND output MATCHES "\n<replacement offset")
        string(SUBSTRING "${file}" "${path_prefix_length}" -1 relative_file)
        list(APPEND badly_formatted "${relative_file}")
    endif()
    set(output "")
endforeach()

if(NOT badly_formatted STREQUAL "")
    list(JOIN badly_formatted "\n" bad_list)
    message("The following files are badly formatted:\n\n${bad_list}\n")
    message(FATAL_ERROR "Run again with FIX=YES to fix these files.")
endif()
