cmake_minimum_required(VERSION 3.10)

# Set the project name
project(mdsplus_library)

# Add the source file
set(SOURCE_FILE mdsplus_library.cpp)

# Create the shared library
add_library(mdsplus_library SHARED ${SOURCE_FILE})

# Set the output name for the shared library
set_target_properties(mdsplus_library PROPERTIES OUTPUT_NAME "mdsplus_library")

# Specify the C++ standard
set_target_properties(mdsplus_library PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED YES)

# Optionally, specify include directories if needed
 target_include_directories(mdsplus_library PRIVATE /usr/local/mdsplus/include)

# Optionally, link additional libraries if needed
# target_link_libraries(mdsplus_library PRIVATE /usr/local/mdsplus/lib/libMdsObjects.so)
# Optionally, set the library version
set_target_properties(mdsplus_library PROPERTIES VERSION 1.0 SOVERSION 1)
