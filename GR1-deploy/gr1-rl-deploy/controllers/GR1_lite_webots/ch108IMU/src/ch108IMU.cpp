#include "ch108IMU.h"

namespace ch108 {
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

int flag;
static raw_t raw;
static int frame_rate;
int frame_count;
int cond_flag;
int baud = 921600;

typedef struct
{
    uint8_t code;
    char name[9];
} item_code_name_t;

const item_code_name_t item_code_name[] =
    {
        {0x90, "id"},
        {0xA0, "acc"},
        {0xB0, "gyr"},
        {0xC0, "mag"},
        {0xD0, "eul"},
        {0xD1, "quat"},
        {0xF0, "pressure"},
        {0x91, "IMUSOL"},
        {0x62, "GWSOL"},
};

void ch108IMU::thread_imudata(int arg) {
    int n = 0;
    int rev = 0;
    int count = 0;
    // int fd = *((int *)arg);
    int fd = arg;
    uint8_t buf[2048] = "";

    // alarm(1);

    while (1) {

        n = read(fd, buf, sizeof(buf));
        // std::cout<<"aaffwcr"<<buf<<std::endl;
        // n = 1;
        count += n;
        if (n > 0) {
            for (int i = 0; i < n; i++) {

                rev = ch_serial_input(&raw, buf[i]);
                // rev = 1;
                // std::cout<<"aaffwcrf"<<rev<<std::endl;
                if (rev) {

                    count = 0;
                    // puts("\033c");
                    frame_count++;
                    dump_data_packet(&raw);
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); //
    }
}

int ch108IMU::open_port(char *port_device) {
    struct termios options;
    int fd = open(port_device, O_RDWR | O_NOCTTY);

    tcgetattr(fd, &options);

    if (fd == -1) {
        perror("open_port: Unable to open SerialPort");
        puts("Please check the usb port name!!!");
        puts("such as \" sudo ./main ttyUSB0 \"");
        exit(0);
    }

    if (fcntl(fd, F_SETFL, 0) < 0) {
        printf("fcntl failed\n");
    } else {
        fcntl(fd, F_SETFL, 0);
    }

    if (isatty(STDIN_FILENO) == 0) {
        printf("standard input is not a terminal device\n");
    } else {
        printf("isatty success!\n");
    }

    bzero(&options, sizeof(options));

    switch (baud) {
    case 9600:
        options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
        break;
    case 115200:
        options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
        break;
    case 230400:
        options.c_cflag = B230400 | CS8 | CLOCAL | CREAD;
        break;
    case 460800:
        options.c_cflag = B460800 | CS8 | CLOCAL | CREAD;
        break;
    case 921600:
        options.c_cflag = B921600 | CS8 | CLOCAL | CREAD;
        break;
    default:
        printf("port baud input error!\r\n");
        exit(0);
    }

    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);
    return (fd);
}

void ch108IMU::initIMU() {

    int fd = 0;
    char dir_usb_dev[64] = "/dev/ttyUSB0";

    // Eigen::Matrix3d tmp1;
    // Eigen::AngleAxisd x_Angle(Eigen::AngleAxisd(a.imuData(2)*M_PI/180,Eigen::Vector3d::UnitX()));
    // Eigen::AngleAxisd y_Angle(Eigen::AngleAxisd(a.imuData(1)*M_PI/180,Eigen::Vector3d::UnitY()));
    // Eigen::AngleAxisd z_Angle(Eigen::AngleAxisd(a.imuData(0)*M_PI/180,Eigen::Vector3d::UnitZ()));
    // tmp1 = x_Angle.toRotationMatrix()* y_Angle.toRotationMatrix()* z_Angle.toRotationMatrix();
    // // // tmp1.eulerAngles(2,1,0)
    // std::cout<<"tmp1: "<<tmp1<<std::endl;

    int i;
    ssize_t n = 0;

    flag = open_port(dir_usb_dev);

    readdata = std::thread(&ch108IMU::thread_imudata, this, flag);
}

void ch108IMU::dump_data_packet(raw_t *raw) {

    Eigen::Vector3d vel;
    Eigen::Matrix3d pos_under_ch108;
    Eigen::Matrix3d pos_under_vn;
    Eigen::AngleAxisd x_Angle(Eigen::AngleAxisd(raw->imu[0].eul[1] * M_PI / 180, Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd y_Angle(Eigen::AngleAxisd(raw->imu[0].eul[0] * M_PI / 180, Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd z_Angle(Eigen::AngleAxisd(raw->imu[0].eul[2] * M_PI / 180, Eigen::Vector3d::UnitZ()));
    pos_under_ch108 = y_Angle * x_Angle * z_Angle;

    Eigen::AngleAxisd z_Angle_1(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
    Eigen::AngleAxisd x_Angle_1(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd z_Angle_2(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()));
    pos_under_vn = z_Angle_2 * x_Angle_1 * pos_under_ch108 * x_Angle_1 * z_Angle_1;

    vel << raw->imu[0].gyr[0], raw->imu[0].gyr[1], raw->imu[0].gyr[2];
    vel = z_Angle_2 * x_Angle_1 * vel;
    // std::cout<<"pos_under_ch108: "<<pos_under_ch108<<std::endl;
    // std::cout<<"trans: "<<x_Angle_1 .toRotationMatrix()* z_Angle_1.toRotationMatrix()<<std::endl;
    // std::cout<<"pos_under_vn: "<<pos_under_vn<<std::endl;

    // std::cout<<"ch108 : "<<rotm2zyx(pos_under_vn)(0) * 180/M_PI<<" "<<rotm2zyx(pos_under_vn)(1) * 180/M_PI<<" "<<rotm2zyx(pos_under_vn)(2) * 180/M_PI<<std::endl;

    imudata << rotm2zyx(pos_under_vn)(0),
        -rotm2zyx(pos_under_vn)(1),
        -rotm2zyx(pos_under_vn)(2),
        -vel(0) * M_PI / 180,
        -vel(1) * M_PI / 180,
        vel(2) * M_PI / 180,
        -raw->imu[0].acc[1] * (-10),
        -raw->imu[0].acc[0] * (-10),
        raw->imu[0].acc[2] * (-10);
}
} // namespace ch108
