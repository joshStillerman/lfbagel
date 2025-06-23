
cmake_minimum_required(VERSION 3.5)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

project(bagel_rpi VERSION 0.2.0)

# set(CMAKE_VERBOSE_MAKEFILE ON)
cmake_policy(SET CMP0060 NEW)

# include(CTest)
# enable_testing()

# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)

include(FetchContent)
FetchContent_Declare(
    daqhats
    GIT_REPOSITORY https://github.com/mccdaq/daqhats.git
    GIT_TAG master
)

FetchContent_MakeAvailable(daqhats)
include_directories(${daqhats_SOURCE_DIR}/examples/c)

find_package(MDSplus REQUIRED)

add_subdirectory(src)
add_subdirectory(etc)
