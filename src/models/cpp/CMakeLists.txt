cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(inference-cpp)

# Set up the C++ Torch package
find_package(Torch REQUIRED)
# This line is functionally equivalent to
# CMAKE_CXX_FLAGS = ${CMAKE_CXX_FLAGS} + ${TORCH_CXX_FLAGS}
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS})

# Set up the OpenCV C++ package
# The variable OpenCV_DIR must be set to the folder containing the file OpenCVConfig.cmake
set(OpenCV_DIR "C:/lib/install/opencv")
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)

# Configure the exe named 'inference' to be built from main.cpp
add_executable(inference main.cpp)

# Set up libraries to be linked
target_link_libraries(inference ${TORCH_LIBRARIES} ${OpenCV_LIBS})

# Set the 'inference' exe to target C++ 17
set_property(TARGET inference PROPERTY CXX_STANDARD 17)

# The following code block is suggested to be used on Windows.
# According to https://github.com/pytorch/pytorch/issues/25457,
# the DLLs need to be copied to avoid memory errors.
if (MSVC)
    file(GLOB TORCH_DLLS "${TORCH_INSTALL_PREFIX}/lib/*.dll")
    add_custom_command(TARGET inference
                        POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        ${TORCH_DLLS}
                        $<TARGET_FILE_DIR:inference>)
endif (MSVC)
