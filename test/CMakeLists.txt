cmake_minimum_required(VERSION 2.8)

project(tests)

enable_language(C)
enable_language(CXX)

set(CMAKE_CXX_FLAGS "-std=c++11")


set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

include_directories(../src) #add .h
include_directories(fake) #add .h

include_directories(mock/include/arduino-mock)

include(CTest)

if (CMAKE_VERSION VERSION_LESS 3.2)
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "")
else()
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "UPDATE_DISCONNECTED 1")
endif()

include(DownloadProject.cmake)
download_project(PROJ                googletest
                 GIT_REPOSITORY      https://github.com/google/googletest.git
                 GIT_TAG             master
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
)

# Prevent GoogleTest from overriding our compiler/linker options
# when building with Visual Studio
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})

# When using CMake 2.8.11 or later, header path dependencies
# are automatically added to the gtest and gmock targets.
# For earlier CMake versions, we have to explicitly add the
# required directories to the header search path ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
    include_directories("${gtest_SOURCE_DIR}/include"
                        "${gmock_SOURCE_DIR}/include")
endif()



# file(GLOB SOURCES ../src/*.cpp)

set(SOURCES
../src/Kinematic.cpp
# ../src/Logger.cpp
)

file(GLOB TEST_FILES *.cpp)
file(GLOB FAKE_FILES fake/*.cpp)
file(GLOB MOCKS mock/src/*.cpp)

add_executable(tests
               ${TEST_FILES}
               ${FAKE_FILES}
               ${SOURCES}
               ${MOCKS}
               )


target_link_libraries(tests gtest gmock_main )
