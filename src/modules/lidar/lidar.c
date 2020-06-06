/*

 */
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <math.h>
#include <uORB/topics/lidar.h>

static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;


__EXPORT int lidar_main(int argc, char *argv[]);
int lidar_thread_main(int argc, char *argv[]);

static int uart_init(const char * uart_name);    //
static int set_uart_baudrate(const int fd, unsigned int baud); 
static void usage(const char *reason);            



int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) 
    {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            warnx("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;


    tcgetattr(fd, &uart_config);

    uart_config.c_oflag &= ~ONLCR;

    uart_config.c_cflag &= ~(CSTOPB | PARENB);

    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) 
    {
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) 
    {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) 
    {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}


int uart_init(const char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) 
    {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    return serial_fd;
}

static void usage(const char *reason)
{
    if (reason) 
    {
        fprintf(stderr, "%s\n", reason);
    }

    fprintf(stderr, "usage: position_estimator_inav {start|stop|status} [param]\n\n");
    exit(1);
}

int lidar_main(int argc, char *argv[])
{
    if (argc < 2) 
    {
        usage("missing command");
    }

    if (!strcmp(argv[1], "start")) 
    {
        if (thread_running) {
            warnx("already running\n");
            exit(0);
        }

        thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd("lidar",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_MAX - 5,
                         3000,
                         lidar_thread_main,
                         (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
        exit(0);
    }

    if (!strcmp(argv[1], "stop")) 
    {
        thread_should_exit = true;
        exit(0);
    }

    if (!strcmp(argv[1], "status")) 
    {
        if (thread_running) 
        {
            warnx("running");

        } 
        else 
        {
            warnx("stopped");
        }

        exit(0);
    }

    usage("unrecognized command");
    exit(1);
}

int lidar_thread_main(int argc, char *argv[])
{

    if (argc < 1) 
    {
        errx(1, "need a serial port name as argument");
        usage("eg:");
    }

/*  TELEM1 : /dev/ttyS1 * 
    TELEM2 : /dev/ttyS2 * 
    GPS : /dev/ttyS3 * 
    NSH : /dev/ttyS5 * 
    SERIAL4: /dev/ttyS6 * 
    N/A : /dev/ttyS4 * 
    IO DEBUG (RX only):/dev/ttyS0 
*/

    const char *uart_name = "/dev/ttyS2";

    warnx("opening port %s", uart_name);
    char FrameHead = 0x0A;

    char data='0';
    // char buffer[5] = {0};   //buffer cache 
    double result1 = 0.0;
    double lidarResult = 0.0;
    double last_lidarResult = 0;



    int uart_read = uart_init(uart_name);
    if(false == uart_read)
    {
        return -1;
    }
    if(false == set_uart_baudrate(uart_read,115200))
    {
        printf("set_uart_baudrate is failed\n");
        return -1;
    }
    printf("uart init is successful\n");
    thread_running = true;


    struct lidar_s lidardate;
    memset(&lidardate, 0 , sizeof(lidardate));
    orb_advert_t lidar_pub = orb_advertise(ORB_ID(lidar), &lidardate);//公告这个主题
    int data_point=0;  //count

    
    while(!thread_should_exit)
    {
        read(uart_read,&data,1);
        if (data == FrameHead)
        {   
            char buffer[5] = {0};   //create new buffer cache each time receive frame
            int k=0;
            data_point = 0;
            for(int i = 0;i <5;++i)
            {
                read(uart_read,&data,1);
                if(data == 0x2E)
                {
                    data_point=i;
                    // i -= 1;
                    continue;
                }
                buffer[k]=data;
                k = k + 1;
            }
            if(data_point ==1 || data_point == 2)
            {
                result1 = 0;   //reset 0 at each frame
                double tmp_res = 0;
                for(int i=0;i<4;i++)
                {
                    tmp_res =(buffer[i]-48)*pow(10,(2-i+data_point));
                    result1 += tmp_res;
                }
                lidarResult = result1/1000.0;
                if (lidarResult <0 || lidarResult>40)
                {
                    lidarResult = last_lidarResult;
                }
                last_lidarResult = lidarResult;
                PX4_INFO("this is a test about lidat measurements At Jun 06  %0.2f m\n",lidarResult);
            }

        }
        lidardate.lidar_result = lidarResult;
        orb_publish(ORB_ID(lidar), lidar_pub, &lidardate);
    }



    warnx("exiting");
    thread_running = false;
    close(uart_read);

    fflush(stdout);
    return 0;
}