
if(NOT PROJECT_NAME)
    cmake_minimum_required(VERSION 3.22)
    include(cmake/EngineFinder.cmake OPTIONAL)
    find_package(o3de REQUIRED)
    project(andino_o3de
        LANGUAGES C CXX
        VERSION 1.0.0.0
    )
    o3de_initialize()
endif()
