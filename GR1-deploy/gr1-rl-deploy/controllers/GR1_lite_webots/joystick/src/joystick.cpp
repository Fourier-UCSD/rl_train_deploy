#include"../include/joystick.h"
#include<iostream>
Joystick_robot::Joystick_robot(){}

Joystick_robot::~Joystick_robot(){}

int Joystick_robot::xbox_open(const char *file_name)  
{  
    int xbox_fd;  

    xbox_fd = open(file_name, O_RDONLY);  
    if (xbox_fd < 0)  
    {  
        perror("open");  
        return -1;  
    }  

    return xbox_fd;  
} 

int Joystick_robot::xbox_map_read(xbox_map_t *map)  
{  
    int len, type, number, value;  
    struct js_event js;  

    len = read(xbox_fd, &js, sizeof(struct js_event));  
    if (len < 0)  
    {  
        perror("read");  
        return -1;  
    }  

    type = js.type;  
    number = js.number;  
    value = js.value;  

    map->time = js.time;  

    if (type == JS_EVENT_BUTTON)  
    {  
        switch (number)  
        {  
            case XBOX_BUTTON_A:  
                map->a = value;  
                break;  

            case XBOX_BUTTON_B:  
                map->b = value;  
                break;  

            case XBOX_BUTTON_X:  
                map->x = value;  
                break;  

            case XBOX_BUTTON_Y:  
                map->y = value;  
                break;  

            case XBOX_BUTTON_LB:  
                map->lb = value;  
                break;  

            case XBOX_BUTTON_RB:  
                map->rb = value;  
                break;  

            case XBOX_BUTTON_START:  
                map->start = value;  
                break;  

            case XBOX_BUTTON_BACK:  
                map->back = value;  
                break;  

            case XBOX_BUTTON_HOME:  
                map->home = value;  
                break;  

            case XBOX_BUTTON_LO:  
                map->lo = value;  
                break;  

            case XBOX_BUTTON_RO:  
                map->ro = value;  
                break;  

            default:  
                break;  
        }  
    }  
    else if (type == JS_EVENT_AXIS)  
    {  
        switch(number)  
        {  
            case XBOX_AXIS_LX:  
                map->lx = value;  
                break;  

            case XBOX_AXIS_LY:  
                map->ly = value;  
                break;  

            case XBOX_AXIS_RX:  
                map->rx = value;  
                break;  

            case XBOX_AXIS_RY:  
                map->ry = value;  
                break;  

            case XBOX_AXIS_LT:  
                map->lt = value;  
                break;  

            case XBOX_AXIS_RT:  
                map->rt = value;  
                break;  

            case XBOX_AXIS_XX:  
                map->xx = value;  
                break;  

            case XBOX_AXIS_YY:  
                map->yy = value;  
                break;  

            default:  
                break;  
        }  
    }  
    else  
    {  
        /* Init do nothing */  
    }  

    return len;
}

void Joystick_robot::xbox_close(void)  
{  
    close(xbox_fd);  
    return;  
}

int Joystick_robot::xbox_init(void)
{
    int len, type;  
    int axis_value, button_value;  
    int number_of_axis, number_of_buttons ;  

    xbox_fd = xbox_open("/dev/input/js0");  
    if(xbox_fd < 0)  
    {  
        return -1;  
    }  

    return 0;
}

int Joystick_robot::xbox_read(xbox_map_t *xbox_m)
{

    int len = xbox_map_read(xbox_m);  
    if (len < 0)  
    {  
        return -1;
    }  
    return 0;
}

void Joystick_robot::init()
{
    int ret = -1;
    xbox_m.a = 0.0;
    xbox_m.b = 0.0;
    xbox_m.x = 0.0;
    xbox_m.y = 0.0;
    xbox_m.lx = 0.0;
    xbox_m.ly = 0.0;
    xbox_m.rx = 0.0;
    xbox_m.ry = 0.0;
    xbox_m.xx = 0.0;
    xbox_m.yy = 0.0;
    xbox_m.lt = -32767;
    current_fsmstate_command = "";
    // ret = pthread_create(&Joystick::xbox_thread, NULL, Joystick::xbox_run, NULL);
    xbox_thread = std::thread(&Joystick_robot::xbox_run,this);
}

void Joystick_robot::xbox_run()
{
    int len,ret;
    
    ret = xbox_init(); 
    if(ret < 0)
    {
        printf("xbox init fail\n");
    }

    while(ret == 0)
    {
        len = xbox_read(&xbox_m);
        if (len < 0)  
        {  
            // usleep(10*1000);  
            continue;  
        }  

        // printf("Time:%8d A:%d B:%d X:%d Y:%d LB:%  m.b, xbox_m.x, xbox_m.y, xbox_m.lb, xbox_m.rb, xbox_m.start, xbox_m.back, xbox_m.home, xbox_m.lo, xbox_m.ro,  
        //         xbox_m.xx, xbox_m.yy, xbox_m.lx, xbox_m.ly, xbox_m.rx, xbox_m.ry, xbox_m.lt, xbox_m.rt);  
        // fflush(stdout);  

        // usleep(10*1000);  
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }       
}

std::string Joystick_robot::get_state_change()
{
    if(xbox_m.a == 1.0){
        current_fsmstate_command = "gotoZero";
        return "gotoZero";
    }else if(xbox_m.b == 1.0){
        return "";
    }else if(xbox_m.x == 1.0){
        current_fsmstate_command = "gotoNLP";
        return "gotoNLP";
    }else if(xbox_m.y == 1.0){
        // return "gotoWalk";
        return "";
    }else{
        return "";
    }

}

std::string Joystick_robot::get_current_state_command()
{
    return current_fsmstate_command;
}

double Joystick_robot::get_walk_x_direction_speed()
{
    int lt_value = xbox_m.lt;
    if(lt_value<1000){
        int x_value = xbox_m.ly;
        if((abs(x_value) > deadarea)&&(abs(x_value)<=maxvalue)){
            if(x_value > 0){
                return ly_dir*maxspeed_x*((double)(abs(x_value)-deadarea)/(maxvalue-deadarea));
            }else{
                return ly_dir*minspeed_x*((double)(abs(x_value)-deadarea)/(maxvalue-deadarea));
            }
        }else{
            return 0.0;
        }
    }else{
        return 0.2;
    }
}

double Joystick_robot::get_walk_y_direction_speed()
{
    int y_value = xbox_m.lx;
    if((abs(y_value) > deadarea)&&(abs(y_value)<=maxvalue)){
        if(y_value > 0){
            return lx_dir*maxspeed_y*((double)(abs(y_value)-deadarea)/(maxvalue-deadarea));
        }else{
            return lx_dir*minspeed_y*((double)(abs(y_value)-deadarea)/(maxvalue-deadarea));
        }
    }else{
        return 0.0;
    }
}

double Joystick_robot::get_walk_yaw_direction_speed()
{
    int yaw_value = xbox_m.rx;
    if((abs(yaw_value) > deadarea)&&(abs(yaw_value)<=maxvalue)){
        if(yaw_value > 0){
            return rx_dir*maxspeed_yaw*((double)(abs(yaw_value)-deadarea)/(maxvalue-deadarea));
        }else{
            return rx_dir*minspeed_yaw*((double)(abs(yaw_value)-deadarea)/(maxvalue-deadarea));
        }
    }else{
        return 0.0;
    }
}

double Joystick_robot::get_walk_x_direction_speed_offset()
{
    int x_value = xbox_m.yy;
    if((x_value<-30000)&&(abs(last_value_x)<500))
    {
        last_value_x = x_value;
        return deltoffset_x;
    }else if((x_value>30000)&&(abs(last_value_x)<500)){
        last_value_x = x_value;
        return -deltoffset_x;
    }else{
        last_value_x = x_value;
        return 0.0;
    }
    

}

double Joystick_robot::get_walk_y_direction_speed_offset()
{
    int y_value = xbox_m.xx;
    if((y_value<-30000)&&(abs(last_value_y)<500))
    {
        last_value_y = y_value;
        return deltoffset_y;
    }else if((y_value>30000)&&(abs(last_value_y)<500)){
        last_value_y = y_value;
        return -deltoffset_y;
    }else{
        last_value_y = y_value;
        return 0.0;
    }
    
}

// get commmand position for stand
double Joystick_robot::get_stand_x_direction_position()
{
    int x_value = xbox_m.ly;
    if((abs(x_value) > deadarea)&&(abs(x_value)<=maxvalue)){
        if(x_value > 0){
            return ly_dir*maxposition_x*((double)(abs(x_value)-deadarea)/(maxvalue-deadarea));
        }else{
            return ly_dir*minposition_x*((double)(abs(x_value)-deadarea)/(maxvalue-deadarea));
        }
    }else{
        return 0.0;
    }
}


double Joystick_robot::get_stand_y_direction_posiiton()
{
    int y_value = xbox_m.lx;
    if((abs(y_value) > deadarea)&&(abs(y_value)<=maxvalue)){
        if(y_value > 0){
            return lx_dir*maxposition_y*((double)(abs(y_value)-deadarea)/(maxvalue-deadarea));
        }else{
            return lx_dir*minposition_y*((double)(abs(y_value)-deadarea)/(maxvalue-deadarea));
        }
    }else{
        return 0.0;
    }
}
 

double Joystick_robot::get_stand_z_direction_posiiton()
{
    int z_value = xbox_m.ry;
    if((abs(z_value) > deadarea)&&(abs(z_value)<=maxvalue)){
        if(z_value > 0){
            return ry_dir*maxposition_z*((double)(abs(z_value)-deadarea)/(maxvalue-deadarea));
        }else{
            return ry_dir*minposition_z*((double)(abs(z_value)-deadarea)/(maxvalue-deadarea));
        }
    }else{
        return 0.0;
    }
}


double Joystick_robot::get_stand_roll_direction_position()
{
    int x_value = xbox_m.yy;
    if((x_value<-30000))
    {
        if(roll_cmd < maxposition_roll)
        {
            roll_cmd += rolldelt_cmd;
        }else{
            roll_cmd = maxposition_roll;
        }       
        return roll_cmd;
    }else if((x_value>30000)){
        if(roll_cmd > minposition_roll)
        {
            roll_cmd -= rolldelt_cmd;
        }else{
            roll_cmd = minposition_roll;
        }       
        return roll_cmd;
    }else{
        return roll_cmd;
    }
}


double Joystick_robot::get_stand_pitch_direction_posiiton()
{
    int x_value = xbox_m.xx;
    if((x_value<-30000))
    {
        if(pitch_cmd < maxposition_pitch)
        {
            pitch_cmd += pitchdelt_cmd;
        }else{
            pitch_cmd = maxposition_pitch;
        }       
        return pitch_cmd;
    }else if((x_value>30000)){
        if(pitch_cmd > minposition_pitch)
        {
            pitch_cmd -= pitchdelt_cmd;
        }else{
            pitch_cmd = minposition_pitch;
        }       
        return pitch_cmd;
    }else{
        return pitch_cmd;
    }
}
