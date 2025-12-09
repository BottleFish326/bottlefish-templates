if (PROJECT_IS_TOP_LEVEL)
    add_compile_options(-fdiagnostics-color)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif ()

option(BOTTLEFISH_OPTIMIZE_FOR_MARCH "Build with -march=native if possible" "native")

include(CheckCXXCompilerFlag)

if (BOTTLEFISH_OPTIMIZE_FOR_MARCH)
    CHECK_CXX_COMPILER_FLAG("-march=${BOTTLEFISH_OPTIMIZE_FOR_MARCH}" COMPILER_SUPPORTS_MARCH)
    message(STATUS "Compiler supports -march=${BOTTLEFISH_OPTIMIZE_FOR_MARCH}: ${COMPILER_SUPPORTS_MARCH}")
endif ()

if (COMPILER_SUPPORTS_MARCH AND NOT BOTTLEFISH_OPTIMIZE_FOR_MARCH STREQUAL "FALSE")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=${BOTTLEFISH_OPTIMIZE_FOR_MARCH}")
endif ()

option(BOTTLEFISH_ENABLE_SANITIZER "Enable AddressSanitizer" OFF)
option(BOTTLEFISH_ENABLE_EULA "Enable EULA acceptance" OFF)
if (BOTTLEFISH_ENABLE_SANITIZER)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")
endif ()

option(BOTTLEFISH_ENABLE_FAST_MATH "Enable fast math" OFF)

if (BOTTLEFISH_ENABLE_FAST_MATH)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math")
endif ()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)


set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")