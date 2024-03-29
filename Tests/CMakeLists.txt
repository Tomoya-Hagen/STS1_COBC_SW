# Parent project does not export its library target, so this CML implicitly depends on being added
# from it, i.e. the testing is done only from the build tree and is not feasible from an install
# location

project(Sts1CobcSwTests LANGUAGES CXX)

# ---- Find libraries ----

find_package_and_notify(Catch2)
find_package_and_notify(etl)
find_rodos()

# ---- Tests ----

add_custom_target(AllTests)
if(CMAKE_SYSTEM_NAME STREQUAL Linux)
    add_subdirectory(GoldenTests EXCLUDE_FROM_ALL)
    add_subdirectory(UnitTests EXCLUDE_FROM_ALL)
    add_dependencies(AllTests AllUnitTests AllGoldenTests)
elseif(CMAKE_SYSTEM_NAME STREQUAL Generic)
    add_subdirectory(HardwareTests EXCLUDE_FROM_ALL)
    add_dependencies(AllTests AllHardwareTests)
endif()

# ---- End-of-file commands ----

add_folders(Tests)
