cmake_minimum_required(VERSION 3.16)

if(POLICY CMP0091)
    cmake_policy(SET CMP0091 NEW)
endif()

project(bspdecompiler LANGUAGES CXX)

add_executable(bspdecompiler
    main.cpp
    bsp.cpp
    decompile.cpp
)

target_compile_features(bspdecompiler PRIVATE cxx_std_17)
target_include_directories(bspdecompiler PRIVATE include)

if(MSVC)
    set_property(TARGET bspdecompiler PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    )
    target_compile_options(bspdecompiler PRIVATE /W4 /permissive-)
elseif(MINGW)
    target_link_options(bspdecompiler PRIVATE
        -static
        -static-libgcc
        -static-libstdc++
    )
else()
    target_compile_options(bspdecompiler PRIVATE -Wall -Wextra -Wpedantic)
endif()
