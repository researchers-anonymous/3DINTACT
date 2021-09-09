cmake_minimum_required(VERSION 3.11)
project(i3d)

# optimization for dev environments
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# project version
set(MAJOR 0)
set(MINOR 1)
set(REMAP_VERSION ${MAJOR}.${MINOR})

# flags
set(CMAKE_CXX_FLAGS "-pthread")

# set cmake modules path
list(APPEND CMAKE_MODULE_PATH "$ENV{CMAKE_MODULE_PATH}")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

option(BUILD_SHARED_LIBS "Build Shared Library" ON)           # default ON

# set path variables
set(PROJECT_DIR ${CMAKE_CURRENT_LIST_DIR})

set(BIN_DIR ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BIN_DIR})

set(SRC_DIR ${PROJECT_DIR}/src)
set(LIB_DIR ${PROJECT_DIR}/libs)
set(BUILD_DIR ${PROJECT_DIR}/build)
set(EXT_DIR ${PROJECT_DIR}/external)
set(INCLUDE_DIR ${PROJECT_DIR}/include)

# tidy project
option(TIDY_PROJECT "Tidy up and optimize code base" OFF)   # default OFF
if(TIDY_PROJECT)
    if(UNIX AND NOT APPLE)
        set(CMAKE_CXX_CLANG_TIDY
            clang-tidy -p pwd
            --checks=-*,readability-*,clang-analyzer-*,performance-*)
    endif()
    set(CMAKE_CXX_FLAGS "-Wall -Wextra")
    set(CMAKE_CXX_FLAGS_DEBUG "-g")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
find_package(Git REQUIRED)
find_package(glog REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gflags REQUIRED)
find_package(Pangolin REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

#add_subdirectory(examples)

# setup Azure-Kinect-Sensor-SDK
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    option(SETUP_K4A_SDK "Initialize the kinect SDK project" ON)  # default ON
    if(SETUP_K4A_SDK)
        execute_process(
            COMMAND /bin/bash ./scripts/init_k4a_sdk.sh -j 12
            WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
    endif()
endif()
set(K4A_SDK ${EXT_DIR}/Azure-Kinect-Sensor-SDK)
set(K4A_LIBRARY ${K4A_SDK}/build)
set(K4A_INCLUDE ${K4A_SDK}/include)
set(K4A_VERSION ${K4A_SDK}/build/src/sdk/include)

# setup libtorch
option(SETUP_LIBTORCH "Setup libtorch" ON)
if(SETUP_LIBTORCH)
    execute_process(
        COMMAND /bin/bash ./scripts/libtorch.sh
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
endif()
set(CMAKE_PREFIX_PATH ${EXT_DIR}/libtorch)
set(CMAKE_CXX_FLAGS
    ${CMAKE_CXX_FLAGS}
    ${TORCH_CXX_FLAGS}
    )
find_package(Torch REQUIRED)

# find header files
set (INCLUDE_DIRS "")
file(GLOB_RECURSE HEADERS
    ${LIB_DIR}/*.h
    ${EXT_DIR}/nanoflann/examples/*.h
    ${EXT_DIR}/nanoflann/include/*.hpp
    )
foreach (HEADER ${HEADERS})
    get_filename_component(DIR ${HEADER} PATH)
    list (APPEND INCLUDE_DIRS ${DIR})
endforeach()
list(REMOVE_DUPLICATES INCLUDE_DIRS)

# find source files
file(GLOB_RECURSE LIBS_SRC
    ${LIB_DIR}/*.cpp
    )

option(BUILD_3DINTACT "Build Project" ON) # default OFF
if(BUILD_3DINTACT)
    # setup target
    add_executable(i3d
        ${EXT_SRC}
        ${LIBS_SRC}
        ${SRC_DIR}/main.cpp
        )

    # include directories
    target_include_directories(i3d PRIVATE
        ${K4A_VERSION}
        ${K4A_INCLUDE}
        ${INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        )

    # link libraries
    target_link_libraries(i3d
        glog
        gflags
        Eigen3::Eigen
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
        ${K4A_LIBRARY}/bin/libk4a.so
        ${TORCH_LIBRARIES}
        )

    # execute target
    option(EXECUTE_TARGET "Execute Project Target" ON) # default OFF
    if(EXECUTE_TARGET)
        add_custom_command(TARGET i3d
            POST_BUILD
            COMMAND ./scripts/main.sh
            WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
    endif()
endif()
