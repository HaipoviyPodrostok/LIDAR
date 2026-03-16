cmake_minimum_required(VERSION 3.24)
project(LIDAR)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

FetchContent_Declare(
    json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
)
FetchContent_MakeAvailable(json)



set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SOURCES
    main.cpp
    source/lidar/lidar_processor.cpp
    source/lidar/deviation_map.cpp
    source/geometry/vector_3d.cpp
    source/geometry/triangle.cpp
    source/geometry/section.cpp
    source/geometry/plane.cpp
    source/geometry/line.cpp
)

add_library(lidar_lib STATIC ${SOURCES})
target_include_directories(lidar_lib PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(lidar_lib PUBLIC nlohmann_json::nlohmann_json)

add_executable(lidar.x ${SOURCES})
target_link_libraries(lidar.x PRIVATE lidar_lib)

target_link_libraries(lidar.x PRIVATE nlohmann_json::nlohmann_json)