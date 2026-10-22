# Makes this library usable directly, instead of needing a wrapper in the
# consuming project: as an ESP-IDF component, or in plain CMake through
# add_subdirectory() or FetchContent as the target algo::impedance.
# Ignored by the Makefile build, which stays the way to build and test this
# library on a host.
if(COMMAND idf_component_register)
    idf_component_register(SRCS "src/imp.c"
                           INCLUDE_DIRS "include")
    return()
endif()

add_library(algo-impedance src/imp.c)
add_library(algo::impedance ALIAS algo-impedance)
target_include_directories(algo-impedance PUBLIC include)
set_target_properties(algo-impedance PROPERTIES C_STANDARD 99)
if(NOT MSVC)
    target_link_libraries(algo-impedance PRIVATE m)
endif()
