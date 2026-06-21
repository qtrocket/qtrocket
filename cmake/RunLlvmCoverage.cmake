foreach(required_var
        COVERAGE_BUILD_DIR
        COVERAGE_SOURCE_DIR
        COVERAGE_BINARIES
        LLVM_COV_EXECUTABLE
        LLVM_PROFDATA_EXECUTABLE
        CTEST_COMMAND)
    if(NOT DEFINED ${required_var} OR "${${required_var}}" STREQUAL "")
        message(FATAL_ERROR "RunLlvmCoverage.cmake requires ${required_var}")
    endif()
endforeach()

if(NOT DEFINED COVERAGE_TEST_REGEX OR "${COVERAGE_TEST_REGEX}" STREQUAL "")
    set(COVERAGE_TEST_REGEX "^qtrocket_.*tests$")
endif()

if(NOT DEFINED COVERAGE_IGNORE_REGEX OR "${COVERAGE_IGNORE_REGEX}" STREQUAL "")
    set(COVERAGE_IGNORE_REGEX ".*/(build|build-coverage|_deps|tests)/.*")
endif()

string(REPLACE "\\;" ";" COVERAGE_BINARIES "${COVERAGE_BINARIES}")

set(COVERAGE_DIR "${COVERAGE_BUILD_DIR}/coverage")
set(COVERAGE_RAW_DIR "${COVERAGE_DIR}/raw")
set(COVERAGE_HTML_DIR "${COVERAGE_DIR}/html")
set(COVERAGE_PROFDATA "${COVERAGE_DIR}/qtrocket.profdata")
set(COVERAGE_SUMMARY "${COVERAGE_DIR}/summary.txt")

file(REMOVE_RECURSE "${COVERAGE_RAW_DIR}" "${COVERAGE_HTML_DIR}")
file(MAKE_DIRECTORY "${COVERAGE_RAW_DIR}" "${COVERAGE_HTML_DIR}")

message(STATUS "Running CTest with LLVM_PROFILE_FILE=${COVERAGE_RAW_DIR}/%p-%m.profraw")
execute_process(
    COMMAND "${CMAKE_COMMAND}" -E env
            "LLVM_PROFILE_FILE=${COVERAGE_RAW_DIR}/%p-%m.profraw"
            "${CTEST_COMMAND}" --output-on-failure -R "${COVERAGE_TEST_REGEX}"
    WORKING_DIRECTORY "${COVERAGE_BUILD_DIR}"
    RESULT_VARIABLE ctest_result)
if(NOT ctest_result EQUAL 0)
    message(FATAL_ERROR "CTest failed while collecting coverage")
endif()

file(GLOB coverage_profiles "${COVERAGE_RAW_DIR}/*.profraw")
list(LENGTH coverage_profiles profile_count)
if(profile_count EQUAL 0)
    message(FATAL_ERROR "No .profraw files were produced in ${COVERAGE_RAW_DIR}")
endif()

message(STATUS "Merging ${profile_count} raw profile file(s)")
execute_process(
    COMMAND "${LLVM_PROFDATA_EXECUTABLE}" merge -sparse
            ${coverage_profiles}
            -o "${COVERAGE_PROFDATA}"
    RESULT_VARIABLE profdata_result)
if(NOT profdata_result EQUAL 0)
    message(FATAL_ERROR "llvm-profdata merge failed")
endif()

set(first_binary "")
set(binary_args "")
foreach(binary IN LISTS COVERAGE_BINARIES)
    if(EXISTS "${binary}")
        if(first_binary STREQUAL "")
            set(first_binary "${binary}")
        else()
            list(APPEND binary_args -object "${binary}")
        endif()
    else()
        message(WARNING "Coverage binary does not exist and will be skipped: ${binary}")
    endif()
endforeach()

if(first_binary STREQUAL "")
    message(FATAL_ERROR "No coverage binaries exist")
endif()

set(ignore_arg "-ignore-filename-regex=${COVERAGE_IGNORE_REGEX}")
set(instr_profile_arg "-instr-profile=${COVERAGE_PROFDATA}")

execute_process(
    COMMAND "${LLVM_COV_EXECUTABLE}" report
            "${first_binary}"
            ${binary_args}
            "${instr_profile_arg}"
            "${ignore_arg}"
            "${COVERAGE_SOURCE_DIR}"
    OUTPUT_VARIABLE coverage_report
    ERROR_VARIABLE coverage_report_error
    RESULT_VARIABLE report_result)
if(NOT report_result EQUAL 0)
    message("${coverage_report_error}")
    message(FATAL_ERROR "llvm-cov report failed")
endif()

file(WRITE "${COVERAGE_SUMMARY}" "${coverage_report}")
message(STATUS "Coverage summary:\n${coverage_report}")

execute_process(
    COMMAND "${LLVM_COV_EXECUTABLE}" show
            "${first_binary}"
            ${binary_args}
            "${instr_profile_arg}"
            "${ignore_arg}"
            -format=html
            "-output-dir=${COVERAGE_HTML_DIR}"
            -show-line-counts-or-regions
            -show-branches=count
            -Xdemangler c++filt
            "${COVERAGE_SOURCE_DIR}"
    ERROR_VARIABLE coverage_show_error
    RESULT_VARIABLE show_result)
if(NOT show_result EQUAL 0)
    message("${coverage_show_error}")
    message(FATAL_ERROR "llvm-cov show failed")
endif()

message(STATUS "Coverage HTML: ${COVERAGE_HTML_DIR}/index.html")
message(STATUS "Coverage summary file: ${COVERAGE_SUMMARY}")
