cmake_minimum_required(VERSION 2.8)

project(tests)

# set(CMAKE_SHARED_LIBRARY_CREATE_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_CREATE_CXX_FLAGS} -undefined dynamic_lookup")

enable_language(C)
enable_language(CXX)

set(CMAKE_CXX_FLAGS "-std=c++11 -stdlib=libc++")

if(CMAKE_CXX_COMPILER_ID MATCHES GNU)
    set(CMAKE_CXX_FLAGS         ${CMAKE_CXX_FLAGS} "-Wall -Wno-unknown-pragmas -Wno-sign-compare -Woverloaded-virtual -Wwrite-strings -Wno-unused")
    set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g3")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
endif()

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})


add_subdirectory(lib/googletest-master)

include_directories(../src) #add .h
include_directories(fake) #add .h

include_directories(mock/include/arduino-mock)
include_directories(lib/googletest-master/googletest/include)
include_directories(lib/googletest-master/googlemock/include)

# file(GLOB SOURCES ../src/*.cpp)

set(SOURCES
../src/Kinematic.cpp
# ../src/Logger.cpp
)

file(GLOB SOURCE_FILES *.cpp)
file(GLOB FAKE_FILES fake/*.cpp)
file(GLOB MOCKS mock/src/*.cpp)

add_executable(tests
               ${SOURCE_FILES}
               ${FAKE_FILES}
               ${SOURCES}
               ${MOCKS}
               )

target_link_libraries(tests gtest gtest_main)
