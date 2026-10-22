# Makes this library usable directly, instead of needing a wrapper in the
# consuming project: as an ESP-IDF component, or in plain CMake through
# add_subdirectory() or FetchContent as the target algo::foc.
# Ignored by the Makefile build, which stays the way to build and test this
# library on a host.
if(COMMAND idf_component_register)
    # upid ships no IDF CMakeLists, so the project has to provide a component
    # named upid itself - see "ESP-IDF" in the repo README.
    idf_component_register(SRCS "src/foc.c" "src/foc_loop.c"
                           INCLUDE_DIRS "include"
                           REQUIRES upid)
    return()
endif()

# upid lives in its own repo and has no build system, so unless the parent
# project already made a target called upid, fetch it at a pinned commit and
# wrap its one .c file here. -DFETCHCONTENT_SOURCE_DIR_UPID=/path/to/upid
# uses a local checkout instead of the network.
if(NOT TARGET upid)
    include(FetchContent)
    FetchContent_Declare(upid
        GIT_REPOSITORY https://github.com/JonasAtelier/upid.git
        GIT_TAG        4d96e5058268b18321e6141a240d43ad65b70d80)
    FetchContent_MakeAvailable(upid)
    add_library(upid ${upid_SOURCE_DIR}/src/upid.c)
    target_include_directories(upid PUBLIC ${upid_SOURCE_DIR}/include)
    set_target_properties(upid PROPERTIES C_STANDARD 99)
    if(NOT MSVC)
        target_link_libraries(upid PRIVATE m)
    endif()
endif()

add_library(algo-foc src/foc.c src/foc_loop.c)
add_library(algo::foc ALIAS algo-foc)
target_include_directories(algo-foc PUBLIC include)
set_target_properties(algo-foc PROPERTIES C_STANDARD 99)
# PUBLIC: the caller creates and tunes the two upid controllers foc runs.
target_link_libraries(algo-foc PUBLIC upid)
if(NOT MSVC)
    target_link_libraries(algo-foc PRIVATE m)
endif()
