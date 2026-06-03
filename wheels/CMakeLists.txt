cmake_minimum_required( VERSION 3.16 )

project( arx5_interface )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON) # global PIC

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(Boost REQUIRED COMPONENTS container)


execute_process(
    COMMAND ${Python_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_path('purelib'))"
    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_package(pybind11 REQUIRED PATHS ${PYTHON_SITE_PACKAGES}/pybind11/share/cmake/pybind11)

find_package(Python3 COMPONENTS Interpreter Development)


add_compile_definitions(SDK_ROOT="${CMAKE_BINARY_DIR}/..")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm.*|arm|aarch64")
    set(LIB_SUBDIR aarch64)
    set(SYSTEM_LIB_DIR /lib/aarch64-linux-gnu)

elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64)|(AMD64)|(^i686)")
    set(LIB_SUBDIR x86_64)
    set(SYSTEM_LIB_DIR /lib/x86_64-linux-gnu)
endif()
set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/arx5_interface/lib/${LIB_SUBDIR})
set(ROS_HUMBLE_PREFIX /opt/ros/humble)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(BEFORE
    /usr/include
    /usr/include/eigen3
    ${ROS_HUMBLE_PREFIX}/include
    ${ROS_HUMBLE_PREFIX}/include/kdl_parser
    ${ROS_HUMBLE_PREFIX}/include/urdfdom_headers
)
include_directories(
    $ENV{CONDA_PREFIX}/include/
    $ENV{CONDA_PREFIX}/include/urdfdom_headers
)

pybind11_add_module(arx5_interface
    ${CMAKE_CURRENT_SOURCE_DIR}/arx5_interface/python/arx5_pybind.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arx5_interface/src/app/joint_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arx5_interface/src/app/cartesian_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arx5_interface/src/app/controller_base.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arx5_interface/src/utils.cpp
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/arx5_interface/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/arx5_interface/include/libcan)
add_compile_options(-pthread)

target_link_directories(arx5_interface PRIVATE
    ${LIB_DIR}
    ${ROS_HUMBLE_PREFIX}/lib
    ${SYSTEM_LIB_DIR}
    $ENV{CONDA_PREFIX}/lib
)

target_link_libraries(arx5_interface PRIVATE
    -Wl,--no-as-needed
    ${ROS_HUMBLE_PREFIX}/lib/librcutils.so
    ${ROS_HUMBLE_PREFIX}/lib/librcpputils.so
    ${ROS_HUMBLE_PREFIX}/lib/libament_index_cpp.so
    ${ROS_HUMBLE_PREFIX}/lib/libclass_loader.so
    ${ROS_HUMBLE_PREFIX}/lib/liburdf.so
    ${ROS_HUMBLE_PREFIX}/lib/libkdl_parser.so
    ${SYSTEM_LIB_DIR}/liborocos-kdl.so.1.5
    -Wl,--as-needed
    hardware
    solver
    spdlog::spdlog
    Eigen3::Eigen
    Threads::Threads
    pthread
)


target_include_directories(arx5_interface PUBLIC ${EIGEN3_INCLUDE_DIRS})

set_target_properties(arx5_interface PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN/../lib/${LIB_SUBDIR};${ROS_HUMBLE_PREFIX}/lib;$ENV{CONDA_PREFIX}/lib;${SYSTEM_LIB_DIR}"
)
if(DEFINED CMAKE_LIBRARY_OUTPUT_DIRECTORY)
    set_target_properties(arx5_interface PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
else()
    set_target_properties(arx5_interface PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/arx5_interface/python")
endif()

# Add this to ensure the wheel is built with the correct ABI flags
if(DEFINED ENV{CONDA_PREFIX})
    set(PYTHON_ABI ".cp${Python_VERSION_MAJOR}${Python_VERSION_MINOR}-cp${Python_VERSION_MAJOR}${Python_VERSION_MINOR}")
else()
    execute_process(
        COMMAND ${Python_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_config_var('SOABI'))"
        OUTPUT_VARIABLE PYTHON_ABI
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
endif()
