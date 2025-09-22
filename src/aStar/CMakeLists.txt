cmake_minimum_required(VERSION 3.5)
project(aStar)

add_subdirectory(../map ${CMAKE_BINARY_DIR}/map_build)

add_library(${PROJECT_NAME} src/a_star.cpp src/hybrid_a_star.cpp)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME} map)
