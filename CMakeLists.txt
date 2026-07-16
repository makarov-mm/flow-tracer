cmake_minimum_required(VERSION 3.16)
project(FlowTracer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Qt6 REQUIRED COMPONENTS Widgets)
find_package(Threads REQUIRED)

add_executable(FlowTracer main.cpp)
target_link_libraries(FlowTracer PRIVATE Qt6::Widgets Threads::Threads)

if(MSVC)
    target_compile_options(FlowTracer PRIVATE
        $<$<NOT:$<CONFIG:Debug>>:/O2>
        /fp:fast)
    set_target_properties(FlowTracer PROPERTIES WIN32_EXECUTABLE ON)
else()
    target_compile_options(FlowTracer PRIVATE
        $<$<NOT:$<CONFIG:Debug>>:-O3 -ffast-math>)
endif()
