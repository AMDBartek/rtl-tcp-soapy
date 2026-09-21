# rtl_tcp_soapy - an rtl_tcp compatible I/Q server backed by SoapySDR

cmake_minimum_required(VERSION 3.7)

project(rtl_tcp_soapy
    VERSION 1.0.0
    DESCRIPTION "An rtl_tcp compatible I/Q server backed by SoapySDR"
    LANGUAGES C)

# Use an optimized build by default.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(GNUInstallDirs)

find_package(Threads REQUIRED)

# Find SoapySDR. Prefer pkg-config, then fall back to the CMake package.
set(SOAPYSDR_TARGET "")
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(SOAPYSDR QUIET IMPORTED_TARGET SoapySDR)
    if(SOAPYSDR_FOUND)
        set(SOAPYSDR_TARGET PkgConfig::SOAPYSDR)
    endif()
endif()
if(NOT SOAPYSDR_TARGET)
    find_package(SoapySDR REQUIRED)
    set(SOAPYSDR_TARGET SoapySDR)
endif()

add_executable(rtl_tcp_soapy src/rtl_tcp_soapy.c)

target_link_libraries(rtl_tcp_soapy PRIVATE
    ${SOAPYSDR_TARGET}
    Threads::Threads)

if(UNIX)
    target_link_libraries(rtl_tcp_soapy PRIVATE m)
endif()

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(rtl_tcp_soapy PRIVATE
        -Wall
        -Wextra
        -Wno-unused-parameter
        -Wno-unused
        -Wsign-compare
        -Wdeclaration-after-statement)
endif()

install(TARGETS rtl_tcp_soapy
    DESTINATION ${CMAKE_INSTALL_BINDIR})

install(FILES rtl_tcp_soapy.1
    DESTINATION ${CMAKE_INSTALL_MANDIR}/man1)
