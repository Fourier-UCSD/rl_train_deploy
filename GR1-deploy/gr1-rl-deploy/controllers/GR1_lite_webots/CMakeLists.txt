cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 17)

# Setup the project.
# Its name is defined to be the controller directory name.
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/libtorch/share/cmake/Torch)
add_definitions(-Wall -O2 -g)
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

# add_definitions(-Wall -std=c++11 -mavx -mfma  -O3  -g )
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

# Get C or C++ sources in the current directory (only).
# file(GLOB C_SOURCES *.c)
# file(GLOB CPP_SOURCES *.cpp ${PROJECT_SOURCE_DIR}/SonnyRobotController/src/*.cpp ${PROJECT_SOURCE_DIR}/MPC_Landing/src/*.cpp ${PROJECT_SOURCE_DIR}/MPC_Gait/src/*.cpp)

# set(SOURCES ${C_SOURCES} ${CPP_SOURCES})
# Link with the Webots controller library.
# link_directories($ENV{WEBOTS_HOME}/lib/controller)
# set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
# include_directories($ENV{WEBOTS_HOME}/include/controller/c $ENV{WEBOTS_HOME}/include/controller/cpp)
# message(STATUS "webots:$ENV{WEBOTS_HOME}")
# SET(WEBOTS_HOME /usr/local/webots/)
# link_directories(${WEBOTS_HOME}/lib/controller)
# set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
# include_directories(${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)
# message(STATUS "webots:${WEBOTS_HOME}")

# include(./cmake/grpc/common.cmake)
# Search for the RBDL include directory and library
# FIND_PACKAGE(RBDL REQUIRED)
# FIND_PACKAGE(Eigen3 3.0.0 REQUIRED)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/ThirdParty/eigen3)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/ThirdParty/rbdl/include)
link_directories(${PROJECT_SOURCE_DIR}/ThirdParty/rbdl/lib)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core REQUIRED)
find_package(Torch REQUIRED)

# Add the include directory to the include paths
INCLUDE_DIRECTORIES(${RBDL_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

link_directories("/usr/local/lib")
INCLUDE_DIRECTORIES("/usr/local/include")
link_directories("/usr/lib")
INCLUDE_DIRECTORIES("/usr/include")

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# #
set(FOURIER_DIR fsa-cpp-sdk)
set(fourier_cpp_sdk_build_dir fsa-cpp-sdk-${CMAKE_BUILD_TYPE})
string(TOLOWER ${fourier_cpp_sdk_build_dir} fourier_cpp_sdk_build_dir)
get_filename_component(fourier_cpp_sdk_build_dir "${CMAKE_CURRENT_BINARY_DIR}/${fourier_cpp_sdk_build_dir}" REALPATH)

add_subdirectory(${FOURIER_DIR} ${fourier_cpp_build_dir})
link_directories(
	${PROJECT_SOURCE_DIR}/fsa-cpp-sdk/lib
)

# #
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/RobotController/include)
aux_source_directory(${PROJECT_SOURCE_DIR}/RobotController/src RobotController_DIR)

# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/MPC_Landing/include)
# aux_source_directory(${PROJECT_SOURCE_DIR}/MPC_Landing/src MPC_Landing_DIR)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/MPC_Gait/include)
aux_source_directory(${PROJECT_SOURCE_DIR}/MPC_Gait/src MPC_Gait_DIR)

include_directories(${PROJECT_SOURCE_DIR}/)
include_directories(${PROJECT_SOURCE_DIR}/fourier-cpp-sdk/fourier/include)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/WalkPlan/include)
aux_source_directory(${PROJECT_SOURCE_DIR}/WalkPlan/src WalkPlan_DIR)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/MotorList/include)
aux_source_directory(${PROJECT_SOURCE_DIR}/MotorList/src MotorList_DIR)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/S2P/include)
aux_source_directory(${PROJECT_SOURCE_DIR}/S2P/src S2P_DIR)

# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/vnIMU/include)
# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/vnIMU/include/vn)
# aux_source_directory(${PROJECT_SOURCE_DIR}/vnIMU/src vnIMU_DIR)
# aux_source_directory(${PROJECT_SOURCE_DIR}/vnIMU/src/vn vnIMU_vn_DIR)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/ch108IMU/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/ch108IMU/include/ch108)
aux_source_directory(${PROJECT_SOURCE_DIR}/ch108IMU/src ch108_DIR)
aux_source_directory(${PROJECT_SOURCE_DIR}/ch108IMU/src/ch108 ch108_ch_DIR)

include_directories(${PROJECT_SOURCE_DIR}/example/include/broccoli)
include_directories(${PROJECT_SOURCE_DIR}/ThirdParty)
include_directories(${PROJECT_SOURCE_DIR}/example/include)
aux_source_directory(${PROJECT_SOURCE_DIR}/example/src JointControl_DIR)

# include_directories(${PROJECT_SOURCE_DIR}/webots_interface/include/broccoli)
# include_directories(${PROJECT_SOURCE_DIR}/webots_interface/include/nlohmann)
# include_directories(${PROJECT_SOURCE_DIR}/webots_interface/include)
# aux_source_directory(${PROJECT_SOURCE_DIR}/webots_interface/src WEBOTS_DIR)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/StateEstimator/include)
aux_source_directory(${PROJECT_SOURCE_DIR}/StateEstimator/src StateEstimator_DIR)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/joystick/include)
aux_source_directory(${PROJECT_SOURCE_DIR}/joystick/src joystick_DIR)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/NlpModel/include)
aux_source_directory(${PROJECT_SOURCE_DIR}/NlpModel/src Nlpmodel_DIR)

# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/WebotsInterface/include)
# aux_source_directory(${PROJECT_SOURCE_DIR}/WebotsInterface/src WebotsInterface_DIR)

# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/GRPC_Server/GRPC_Sonny/include)
# aux_source_directory(${PROJECT_SOURCE_DIR}/GRPC_Server/GRPC_Sonny/src GRPC_SONNY_DIR)
# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/GRPC_Server/proto_autogen)
# aux_source_directory(${PROJECT_SOURCE_DIR}/GRPC_Server/proto_autogen/fourierdyn/api/sonnie SONNIE_DIR)
# INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/ThirdParty/grpc/x86_64/include)
# Setup the target executable.
add_executable(${PROJECT}
	${RobotController_DIR}
	${MPC_Gait_DIR} ${WalkPlan_DIR}
	${MotorList_DIR}
	${S2P_DIR}

	# ${vnIMU_DIR} ${vnIMU_vn_DIR}
	${ch108_DIR} ${ch108_ch_DIR}
	${JointControl_DIR}
	${StateEstimator_DIR}
	${joystick_DIR}
	${Nlpmodel_DIR}
)

target_link_libraries(${PROJECT} "${TORCH_LIBRARIES}")
set_property(TARGET ${PROJECT} PROPERTY CXX_STANDARD 17)

target_link_libraries(${PROJECT} ${LIBRARIES} Threads::Threads fsasdk rbdl)

# And link the library against the executable
TARGET_LINK_LIBRARIES(${PROJECT}

	# ${PROJECT_SOURCE_DIR}/RobotController/RobotController/lib/libControllerLib.a
	# ${PROJECT_SOURCE_DIR}/RobotController/rbdl/lib/librbdl.a
	# ${PROJECT_SOURCE_DIR}/RobotController/qpOASES/libqpOASES.a
	# ${RBDL_LIBRARY}
	libQt5Core.so.5.15.2

	# libqpOASES.a
)

TARGET_LINK_LIBRARIES(${PROJECT} Qt${QT_VERSION_MAJOR}::Core)

# Copy the target executable at the right location.
add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
	copy ${CMAKE_BINARY_DIR}/${PROJECT} ${CMAKE_SOURCE_DIR}
)
