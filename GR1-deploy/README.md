# GR1-sim2real
Sim2Real code of GR1T2 in nvidia






only ubuntu 20.04!!!!!!!!!!!!!!!!!!
only joystick can be use !!!!!!!




Use steps：

choose which one you want use 

if you want use webots to sim:


1. put your pt model in "PythonModule/source" and change path in "gr1-rl-deploy/controllers/GR1_lite_webots/MPC_Gait/include/GaitGenerator.h"
2. change "CMakeLists_webots.txt" to "CMakeLists.txt"
3. open "#define WEBOTS"and close "#define REALROBOTS" both in "gr1-rl-deploy/controllers/GR1_lite_webots/example/src/main.cpp" and "gr1-rl-deploy/controllers/GR1_lite_webots/MPC_Gait/include/GaitGenerator.h"
4. mkdir build
5. sh autobuild.sh
6. open GR1-sim2real/gr1-rl-deploy/worlds/GR1T1L.wbt with webots
7. connect joystick
8. press A switch to zero
9. press X switch to  RL
10. enjoy your rl!


if you want use realrobot:



1. put your pt model in "PythonModule/source" and change path in "gr1-rl-deploy/controllers/GR1_lite_webots/MPC_Gait/include/GaitGenerator.h"
2. change "CMakeLists_real.txt" to "CMakeLists.txt"
3. open "#define REALROBOTS"and close "#define WEBOTS" both in "gr1-rl-deploy/controllers/GR1_lite_webots/example/src/main.cpp" and "gr1-rl-deploy/controllers/GR1_lite_webots/MPC_Gait/include/GaitGenerator.h"
4. mkdir build
5. sh autobuild.sh
6. change "absolute_pos_zero" in "gr1-rl-deploy/controllers/GR1_lite_webots/MotorList/sources/motorlist.json", correct value need copy from "home/Rocs/bin//MotorList/sources/motorlist.json"   (VERY  IMPORTANT!!)
7. connect joystick
8. press A switch to zero
9. press X switch to  RL
10. enjoy your rl!
