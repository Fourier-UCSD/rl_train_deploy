cmake_minimum_required(VERSION 3.13.1)

set(CMAKE_CXX_STANDARD 17)
project(fsasdk)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/ThirdParty)
include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/Eigen)

aux_source_directory(${PROJECT_SOURCE_DIR}/src dir_fsa)
add_library(fsasdk SHARED ${dir_fsa})

target_link_libraries(fsasdk pthread)

install(TARGETS fsasdk DESTINATION ${PROJECT_SOURCE_DIR}/lib)

# install(DIRECTORY ${PROJECT_SOURCE_DIR}/include DESTINATION ${PROJECT_SOURCE_DIR}/install)