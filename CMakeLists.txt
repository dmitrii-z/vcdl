cmake_minimum_required(VERSION 3.10)

project(SilAdapterTest DESCRIPTION "SiL-Adapter test" LANGUAGES CXX)

find_package(OpenCV REQUIRED)
find_package(libssh REQUIRED)

set(project_INCLUDE_DIRS
    "../SilAdapter/Dependencies/Cla/include"
    "../SilAdapter/TypeLib/include"
    ${OpenCV_INCLUDE_DIRS}
    ${LIBSSH_INCLUDE_DIR}
)

include_directories(${project_INCLUDE_DIRS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../../bin")

add_executable(test
    test.cpp
)

set_property(TARGET test PROPERTY CXX_STANDARD 17)

message(${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(test ${OpenCV_LIBS} ssh)
target_link_libraries(test 
    ${CMAKE_CURRENT_SOURCE_DIR}/../SilAdapter/build/TypeLib/libTypeLib.a
    ${CMAKE_CURRENT_SOURCE_DIR}/../SilAdapter/build/Dependencies/Cla/libCla.a
    ${CMAKE_CURRENT_SOURCE_DIR}/../SilAdapter/build/Release/libSilKit.a
    ${CMAKE_CURRENT_SOURCE_DIR}/../SilAdapter/build/sk/ThirdParty/yaml-cpp/libyaml-cpp.a
    ${CMAKE_CURRENT_SOURCE_DIR}/../SilAdapter/build/ThirdParty/spdlog/libspdlog.a
)

#add_dependencies(test TypeLib)

