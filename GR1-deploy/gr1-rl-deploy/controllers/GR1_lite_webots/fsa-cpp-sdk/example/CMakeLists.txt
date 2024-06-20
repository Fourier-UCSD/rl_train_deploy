cmake_minimum_required(VERSION 3.16.1)

set(CMAKE_CXX_STANDARD 17)
project(main)

include_directories(${PROJECT_SOURCE_DIR}/../include)
include_directories(../ThirdParty/)
include_directories(include/)
include_directories(include/nlohmann)
link_directories(${PROJECT_SOURCE_DIR}/../lib)

include_directories(${PROJECT_SOURCE_DIR}/../ThirdParty)
include_directories(${PROJECT_SOURCE_DIR}/../ThirdParty/Eigen)

add_executable(main

#     # ${PROJECT_SOURCE_DIR}/demo_muti_motor.cpp
#     # ${PROJECT_SOURCE_DIR}/demo_threadpool_test.cpp
#     # ${PROJECT_SOURCE_DIR}/all_motor_test.cpp

    # ${PROJECT_SOURCE_DIR}/demo_muti_motor.cpp
    # ${PROJECT_SOURCE_DIR}/demo_threadpool_test.cpp
    # ${PROJECT_SOURCE_DIR}/all_motor_test.cpp
    ${PROJECT_SOURCE_DIR}/src/demo_pd_iden.cpp

# # ${PROJECT_SOURCE_DIR}/demo_muti_motor.cpp
# # ${PROJECT_SOURCE_DIR}/demo_vel_ff.cpp
)

# target_link_libraries(main fsasdk pthread)
# add_executable(demo_current_ff src/demo_current_ff.cpp)
target_link_libraries(main fsasdk pthread)