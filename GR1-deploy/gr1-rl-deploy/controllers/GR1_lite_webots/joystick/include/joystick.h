#ifndef JOYSTICK_H_
#define JOYSTICK_H_
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>

#include <linux/input.h>
#include <linux/joystick.h>
// #include "xbox.h"
#define XBOX_TYPE_BUTTON 0x01
#define XBOX_TYPE_AXIS 0x02

#define XBOX_BUTTON_A 0x00
#define XBOX_BUTTON_B 0x01
#define XBOX_BUTTON_X 0x02
#define XBOX_BUTTON_Y 0x03
#define XBOX_BUTTON_LB 0x04
#define XBOX_BUTTON_RB 0x05
#define XBOX_BUTTON_START 0x06
#define XBOX_BUTTON_BACK 0x07
#define XBOX_BUTTON_HOME 0x08
#define XBOX_BUTTON_LO 0x09 /* 左摇杆按键 */
#define XBOX_BUTTON_RO 0x0a /* 右摇杆按键 */

#define XBOX_BUTTON_ON 0x01
#define XBOX_BUTTON_OFF 0x00

#define XBOX_AXIS_LX 0x00 /* 左摇杆X轴 */
#define XBOX_AXIS_LY 0x01 /* 左摇杆Y轴 */
#define XBOX_AXIS_RX 0x03 /* 右摇杆X轴 */
#define XBOX_AXIS_RY 0x04 /* 右摇杆Y轴 */
#define XBOX_AXIS_LT 0x02
#define XBOX_AXIS_RT 0x05
#define XBOX_AXIS_XX 0x06 /* 方向键X轴 */
#define XBOX_AXIS_YY 0x07 /* 方向键Y轴 */

#define XBOX_AXIS_VAL_UP -32767
#define XBOX_AXIS_VAL_DOWN 32767
#define XBOX_AXIS_VAL_LEFT -32767
#define XBOX_AXIS_VAL_RIGHT 32767

#define XBOX_AXIS_VAL_MIN -32767
#define XBOX_AXIS_VAL_MAX 32767
#define XBOX_AXIS_VAL_MID 0x00
/**
 * @brief joystick:
 * author : rxy
 * email : rxy19940622@126.com
 */
typedef struct xbox_map {
    int time;
    int a;
    int b;
    int x;
    int y;
    int lb;
    int rb;
    int start;
    int back;
    int home;
    int lo;
    int ro;

    int lx;
    int ly;
    int rx;
    int ry;
    int lt;
    int rt;
    int xx;
    int yy;

} xbox_map_t;

class Joystick_robot {
  public:
    Joystick_robot();
    ~Joystick_robot();
    void init();
    void xbox_run();
    /**
     * @brief Get the state change object
     * A: gotoZero
     * B:
     * X: gotoStand
     * Y: gotoWalk
     * @return std::string
     */
    std::string get_state_change();
    std::string get_current_state_command();
    /**
     * @brief
     *
     */
    int deadarea = 50;
    int maxvalue = 32767;
    double lx_dir = -1.0;
    double ly_dir = -1.0;

    double rx_dir = -1.0;
    double ry_dir = -1.0;

    double xx_dir = -1.0;
    double yy_dir = -1.0;
    // get command  velocity for walk
    double get_walk_x_direction_speed();
    double maxspeed_x = 0.5;  // - x
    double minspeed_x = -0.5; // +x

    double get_walk_y_direction_speed();
    double maxspeed_y = 0;
    double minspeed_y = 0;

    double get_walk_yaw_direction_speed();
    double maxspeed_yaw = 0.5;
    double minspeed_yaw = -0.5;

    double get_walk_x_direction_speed_offset();
    double deltoffset_x = 0.05;
    double last_value_x = 0;

    double get_walk_y_direction_speed_offset();
    double deltoffset_y = 0.10;
    double last_value_y = 0;

    // get commmand position for stand
    double get_stand_x_direction_position();
    double maxposition_x = 0.1;
    double minposition_x = -0.1;

    double get_stand_y_direction_posiiton();
    double maxposition_y = 0.06;
    double minposition_y = -0.06;

    double get_stand_z_direction_posiiton();
    double maxposition_z = 0.2;
    double minposition_z = -0.05;

    double get_stand_roll_direction_position();
    double maxposition_roll = 1.0;
    double minposition_roll = -1.0;
    double rolldelt_cmd = 0.01;
    double roll_cmd = 0.0;

    double get_stand_pitch_direction_posiiton();
    double maxposition_pitch = 0.5;
    double minposition_pitch = -0.5;
    double pitchdelt_cmd = 0.01;
    double pitch_cmd = 0.0;

  private:
    // fsmstate
    std::string current_fsmstate_command;
    //
    xbox_map_t xbox_m;
    // pthread_t xbox_thread;
    std::thread xbox_thread;
    // xbox
    int xbox_fd;
    int xbox_open(const char *file_name);
    int xbox_map_read(xbox_map_t *map);
    void xbox_close(void);
    int xbox_init(void);
    int xbox_read(xbox_map_t *xbox_m);
};
#endif