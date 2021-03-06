/*
 *  RPLIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include <pthread.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#include "comm_zmq.hpp"

#include "robot_detect.hpp"

using namespace goldobot;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif


pthread_t g_slave0_thread;
void *g_slave0_proc(void*);
bool g_slave0_running(void);
bool slave0_stop = false;

pthread_t g_slave1_thread;
void *g_slave1_proc(void*);

pthread_t g_slave2_thread;
void *g_slave2_proc(void*);


extern "C" {
    extern unsigned int g_odo_thread_time_ms;
    extern unsigned int g_odo_time_ms;
    extern int          g_odo_x_mm;
    extern int          g_odo_y_mm;
    extern double       g_odo_theta_deg;
}

double       g_speed_abs = 0.0;
bool         g_forward_move = true;

unsigned int g_main_thread_time_ms_old;
unsigned int g_main_thread_time_ms_delta;
unsigned int g_main_thread_time_ms_delta_max;

unsigned int g_odo_thread_time_ms_old;
unsigned int g_odo_time_ms_old;
int          g_odo_x_mm_old;
int          g_odo_y_mm_old;
double       g_odo_theta_deg_old;
double       g_odo_theta_rad;

unsigned int g_odo_thread_time_ms_delta;
unsigned int g_odo_time_ms_delta;
int          g_odo_x_mm_delta;
int          g_odo_y_mm_delta;
int          g_odo_d_mm_delta;
double       g_odo_theta_deg_delta;

unsigned int g_odo_thread_time_ms_delta_max;
unsigned int g_odo_time_ms_delta_max;
int          g_odo_d_mm_delta_max;
double       g_odo_theta_deg_delta_max;

int dbg_cnt = 0;

float my_x[720];
float my_y[720];
#define SEND_BUF_SIZE (65500) /* taille max datagramme udp */
unsigned char send_buf[SEND_BUF_SIZE];

int my_sock;
struct sockaddr_in viewer_saddr;

int init_sock(char *viewer_address_str)
{
    unsigned int ab0, ab1, ab2, ab3;

    if (sscanf (viewer_address_str, "%d.%d.%d.%d", 
                &ab3, &ab2, &ab1, &ab0) != 4) {
        printf(" error : cannot parse viewer addr (%s)\n", viewer_address_str);
        return -1;
    }

    my_sock = socket (AF_INET, SOCK_DGRAM, 0);

    viewer_saddr.sin_family= AF_INET;
    viewer_saddr.sin_port= htons(1413);
    viewer_saddr.sin_addr.s_addr= htonl((ab3<<24)|(ab2<<16)|(ab1<<8)|(ab0));

    return 0;
}

int send_to_viewer()
{
    unsigned int *cur_ptr_w = (unsigned int *)((void *)(&send_buf[0]));
    int bytes_to_send = 0;
    int g_odo_theta_deg_0_01 = 0;

    /* header tlv */
    *cur_ptr_w = htonl(0x31337000);
    cur_ptr_w++; bytes_to_send+=4;
    *cur_ptr_w = htonl(0); /* longueur totale : recalcule + bas */
    cur_ptr_w++; bytes_to_send+=4;
    /* on reserve l'id 0x31337001 pour "header etendu" */
    /* odometrie */
    *cur_ptr_w = htonl(0x31337002);
    cur_ptr_w++; bytes_to_send+=4;
    *cur_ptr_w = htonl(g_odo_x_mm);
    cur_ptr_w++; bytes_to_send+=4;
    *cur_ptr_w = htonl(g_odo_y_mm);
    cur_ptr_w++; bytes_to_send+=4;
    g_odo_theta_deg_0_01 = g_odo_theta_deg*100.0;
    *cur_ptr_w = htonl(g_odo_theta_deg_0_01);
    cur_ptr_w++; bytes_to_send+=4;
    /* points du scan lidar */
    *cur_ptr_w = htonl(0x31337003);
    cur_ptr_w++; bytes_to_send+=4;
    for (int i=0; i<720; i++) {
        int my_x_int = my_x[i];
        int my_y_int = my_y[i];

        *cur_ptr_w = (unsigned int) my_x_int;
        cur_ptr_w++; bytes_to_send+=4;
        *cur_ptr_w = (unsigned int) my_y_int;
        cur_ptr_w++; bytes_to_send+=4;
    }
    /* marqueur de fin */
    *cur_ptr_w = htonl(0x313370ff);
    cur_ptr_w++; bytes_to_send+=4;

    cur_ptr_w = (unsigned int *)((void *)(&send_buf[0]));
    cur_ptr_w[1] = bytes_to_send;

    int ret = sendto (my_sock, (void *)send_buf, bytes_to_send, 0, 
                      (const sockaddr *) &viewer_saddr, 
                      sizeof (struct sockaddr_in));
    if (ret!=bytes_to_send) {
        printf ("sendto() failed\n");
        return -1;
    }

    return 0;
}


using namespace rp::standalone::rplidar;

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
#if 0 /* FIXME : DEBUG : HACK GOLDO ++ */
        printf("RPLidar health status : %d\n", healthinfo.status);
#endif /* FIXME : DEBUG : HACK GOLDO -- */
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

#include <signal.h>
bool ctrl_c_pressed = false;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

#define MAX(a,b) ((a>b)?a:b)

int main(int argc, const char * argv[]) {
    const char * opt_com_path = NULL;
    _u32         opt_com_baudrate = 115200;
    u_result     op_result;
    int          ret=0;

    dbg_cnt = 0;

    char viewer_addr_str[40];
    double theta_correction = 0.0f;

    ctrl_c_pressed = false;

    g_speed_abs = 0.0;
    g_forward_move = true;

    g_main_thread_time_ms_old = 0;
    g_main_thread_time_ms_delta = 0;
    g_main_thread_time_ms_delta_max = 0;

    g_odo_thread_time_ms_old = 0;
    g_odo_time_ms_old = 0;
    g_odo_x_mm_old = 0;
    g_odo_y_mm_old = 0;
    g_odo_theta_deg_old = 0.0;
    g_odo_theta_rad = 0.0;

    g_odo_thread_time_ms_delta = 0;
    g_odo_time_ms_delta = 0;
    g_odo_x_mm_delta = 0;
    g_odo_y_mm_delta = 0;
    g_odo_d_mm_delta = 0;
    g_odo_theta_deg_delta = 0.0;

    g_odo_thread_time_ms_delta_max = 0;
    g_odo_time_ms_delta_max = 0;
    g_odo_d_mm_delta_max = 0;
    g_odo_theta_deg_delta_max = 0.0;


    printf("Ultra simple LIDAR data grabber for RPLIDAR.\n"
           "Version: Goldo EXPERIMENTAL\n");

    // read viewer address
    if (argc>1) strncpy(viewer_addr_str,argv[1],40);
    else strncpy(viewer_addr_str,"192.168.0.76"/*"127.0.0.1"*/,40);
    printf ("viewer_addr = %s\n", viewer_addr_str);

    // read theta correction
    if (argc>2) {
        theta_correction = strtod(argv[2], NULL);
        theta_correction = theta_correction*M_PI/180.0f;
    } else {
        theta_correction = 30.0f; /* PR 28/05/2019 */
        theta_correction = theta_correction*M_PI/180.0f;
    }
    printf ("theta_correction = %f\n", theta_correction*180.0f/M_PI);

    // read serial port from the command line...
    if (argc>3) opt_com_path = argv[3]; // or set to a fixed value: e.g. "com3" 

    // read baud rate from the command line if specified...
    if (argc>4) opt_com_baudrate = strtoul(argv[4], NULL, 10);

    if (init_sock(viewer_addr_str)<0)
        return -1;


    if (!opt_com_path) {
        opt_com_path = "/dev/rplidar";
    }

    // create the driver instance
    RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }


    // make connection...
    if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                , opt_com_path);
        goto on_finished;
    }

    rplidar_response_device_info_t devinfo;

    // retrieving the device info
    ////////////////////////////////////////
    op_result = drv->getDeviceInfo(devinfo);

    if (IS_FAIL(op_result)) {
        fprintf(stderr, "Error, cannot get device info.\n");
        goto on_finished;
    }

    // print out the device serial number, firmware and hardware version number
#if 0 /* FIXME : DEBUG : HACK GOLDO ++ */
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
           "Firmware Ver: %d.%02d\n"
           "Hardware Rev: %d\n"
           , devinfo.firmware_version>>8
           , devinfo.firmware_version & 0xFF
           , (int)devinfo.hardware_version);
#endif /* FIXME : DEBUG : HACK GOLDO -- */


    // check health...
    if (!checkRPLIDARHealth(drv)) {
        goto on_finished;
    }

    signal(SIGINT, ctrlc);

    drv->startMotor();
    // start scan...
    drv->startScan();


    ret=pthread_create(&g_slave0_thread, NULL, &g_slave0_proc, NULL);
    if(ret!=0) {
        printf("Unable to create slave0 thread\n");
        exit(-3);
    }

    ret=pthread_create(&g_slave1_thread, NULL, &g_slave1_proc, NULL);
    if(ret!=0) {
        printf("Unable to create slave1 thread\n");
        exit(-3);
    }

    ret=pthread_create(&g_slave2_thread, NULL, &g_slave2_proc, NULL);
    if(ret!=0) {
        printf("Unable to create slave2 thread\n");
        exit(-3);
    }


    // fetch result
    while (1) {
        bool   obstacle_detect = false;

        rplidar_response_measurement_node_t nodes[360*2];
        size_t   count = _countof(nodes);

        for (int pos = 0; pos < 360*2; ++pos) {
            my_x[pos] = 0.0;
            my_y[pos] = 0.0;
        }

        RobotDetect::instance().clearSlots();

        op_result = drv->grabScanData(nodes, count);

        if (IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);

            for (int pos = 0; pos < (int)count ; ++pos) {
                double my_theta = ((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f)*(2.0f*M_PI/360.0f) + theta_correction;
                my_theta = -my_theta; /* FIXME : TODO : explication? (WTF?!) */
                double my_R = nodes[pos].distance_q2/4.0f;

                if ((my_R < 1.0f) || (my_R > 3000.0f)) my_R = 0.0f;

                my_x[pos] = my_R * cos (my_theta);
                my_y[pos] = my_R * sin (my_theta);

                g_odo_theta_rad = g_odo_theta_deg*M_PI/180.0;

                double my_abs_x = my_R * cos (my_theta + g_odo_theta_rad) + g_odo_x_mm;
                double my_abs_y = my_R * sin (my_theta + g_odo_theta_rad) + g_odo_y_mm;

                /* minimalist obstacle detection */ 
                if(
                   (
                    (
                     (my_x[pos]>90.0)&& (my_x[pos]<300.0)&& (g_forward_move)
                    ) 
                    ||
                    (
                     (my_x[pos]<-90.0)&& (my_x[pos]>-300.0)&& (!g_forward_move)
                    )
                   ) 
                   && 
                   (
                    (
                     (my_y[pos]>-140.0) && (my_y[pos]<140.0)
                    )
                   )
                  ) 
                {
                    //printf("GOLDO my_theta:%f my_R:%f my_x[pos]:%f my_y[pos]:%f\n", my_theta, my_R, my_x[pos], my_y[pos]);

                    if ((my_abs_x >   100.0) && (my_abs_x < 1400.0) && 
                        (my_abs_y > -1400.0) && (my_abs_y < 1400.0)) {
                        obstacle_detect = true;
                    }
                }

                if ((my_abs_x >   100.0) && (my_abs_x < 1600.0) && 
                    (my_abs_y > -1400.0) && (my_abs_y < 1400.0) && 
                    (my_R > 100.0) ) {
                    RobotDetect::instance().processNewRplidarSample(g_main_thread_time_ms_old, my_abs_x, my_abs_y);
                }
            }

            RobotDetect::instance().updateDetection();

            RobotDetect::instance().sendDetected();

            send_to_viewer();

#if 1 /* FIXME : DEBUG : HACK GOLDO : minimalist obstacle detection */ 
            {
                int goldo_detect_fd;
                char write_buf[8];
                int res;
                goldo_detect_fd = open ("/sys/class/gpio/gpio21/value", O_RDWR);
                if (goldo_detect_fd<0) {
                    printf ("error opening gpio\n");
                    goto stop_device;
                }
                if (obstacle_detect)
                    write_buf[0] = '1'; 
                else 
                    write_buf[0] = '0';
                write_buf[1] = '\0';
                res = write (goldo_detect_fd,write_buf,1);
                if (res<0) {
                    printf ("error writting gpio value\n");
                }
                close (goldo_detect_fd);
            }
#endif

        }

        if ((dbg_cnt%10)==0) {
            struct timespec my_tp;
            unsigned int main_thread_time_ms;

            /* FIXME : TODO : section critique! */
            volatile unsigned int my_odo_thread_time_ms = g_odo_thread_time_ms;
            volatile unsigned int my_odo_time_ms = g_odo_time_ms;
            volatile int          my_odo_x_mm = g_odo_x_mm;
            volatile int          my_odo_y_mm = g_odo_y_mm;
            volatile double       my_odo_theta_deg = g_odo_theta_deg;

            clock_gettime(1, &my_tp);

            main_thread_time_ms = my_tp.tv_sec*1000 + my_tp.tv_nsec/1000000;

            if (g_main_thread_time_ms_old != 0) {
                g_main_thread_time_ms_delta = main_thread_time_ms - g_main_thread_time_ms_old;
                g_main_thread_time_ms_delta_max = MAX(g_main_thread_time_ms_delta_max, g_main_thread_time_ms_delta);
                g_main_thread_time_ms_old = main_thread_time_ms;

                g_odo_thread_time_ms_delta = abs(my_odo_thread_time_ms - g_odo_thread_time_ms_old);
                g_odo_time_ms_delta = abs(my_odo_time_ms - g_odo_time_ms_old);
                g_odo_x_mm_delta = my_odo_x_mm - g_odo_x_mm_old;
                g_odo_y_mm_delta = my_odo_y_mm - g_odo_y_mm_old;
                g_odo_d_mm_delta = sqrt(g_odo_x_mm_delta*g_odo_x_mm_delta + g_odo_y_mm_delta*g_odo_y_mm_delta);
                g_odo_theta_deg_delta = fabs(my_odo_theta_deg - g_odo_theta_deg_old);

                g_odo_thread_time_ms_delta_max = MAX(g_odo_thread_time_ms_delta_max, g_odo_thread_time_ms_delta);
                g_odo_time_ms_delta_max = MAX(g_odo_time_ms_delta_max, g_odo_time_ms_delta);
                g_odo_d_mm_delta_max = MAX(g_odo_d_mm_delta_max, g_odo_d_mm_delta);
                g_odo_theta_deg_delta_max = MAX(g_odo_theta_deg_delta_max, g_odo_theta_deg_delta);

                g_odo_thread_time_ms_old = my_odo_thread_time_ms;
                g_odo_time_ms_old = my_odo_time_ms;
                g_odo_x_mm_old = my_odo_x_mm;
                g_odo_y_mm_old = my_odo_y_mm;
                g_odo_theta_deg_old = my_odo_theta_deg;

                if (g_odo_time_ms_delta==0) g_speed_abs = 0.0;
                else g_speed_abs = ((double)g_odo_d_mm_delta/*/1000.0*/)/(g_odo_time_ms_delta/*/1000.0*/); // result in m/sec
                if (g_speed_abs>0.005) {
                    g_forward_move = ((g_odo_x_mm_delta*cos(g_odo_theta_rad) + g_odo_y_mm_delta*sin(g_odo_theta_rad)) > 0.0);
                }
            } else {
                g_main_thread_time_ms_old = main_thread_time_ms;
                g_odo_thread_time_ms_old = my_odo_thread_time_ms;
                g_odo_time_ms_old = my_odo_time_ms;
                g_odo_x_mm_old = my_odo_x_mm;
                g_odo_y_mm_old = my_odo_y_mm;
                g_odo_theta_deg_old = my_odo_theta_deg;
            }

#if 0 /* FIXME : DEBUG : USEFULL (don't remove!) */
            printf ("main_thread_time_ms = %u (%u)\n", main_thread_time_ms, g_main_thread_time_ms_delta_max);
            printf ("my_odo_thread_time_ms = %u (%u)\n", my_odo_thread_time_ms, g_odo_thread_time_ms_delta_max);
            printf ("ts = %u (%u); pos = < %d , %d > (%u); theta = %f (%f)\n",
                    my_odo_time_ms, g_odo_time_ms_delta_max, 
                    my_odo_x_mm, my_odo_y_mm, g_odo_d_mm_delta_max,
                    my_odo_theta_deg, g_odo_theta_deg_delta_max);

            printf ("MOVE : %s %f m/s\n", g_forward_move?"FORWARDS":"BACKWARDS", g_speed_abs);

            printf ("\n");
#endif
        }
        dbg_cnt++;

        if (ctrl_c_pressed || (!g_slave0_running())) { 
            slave0_stop = true;

            CommZmq::instance().stopTask();

            printf ("Bye!\n");
            break;
        }
    }

stop_device:
    drv->stop();
    drv->stopMotor();
    // done!
on_finished:
    RPlidarDriver::DisposeDriver(drv);
    return 0;
}


extern "C" {
    extern int read_odo_flag_running;
    extern void read_odo_main(void);
}

void *g_slave0_proc(void*)
{
    printf ("g_slave0_proc()..\n");

#if 1 /* FIXME : DEBUG : 0 pour desactiver le thread odo temps reel */
    read_odo_main();
#else
    while (!slave0_stop) {
        pthread_yield();
    }
#endif

    return NULL;
}

bool g_slave0_running(void)
{
    return (read_odo_flag_running!=0);
}


void *g_slave1_proc(void*)
{
    const char * rplidar_serv_port_str = NULL;
    int rplidar_serv_port = 3101;
    int rc;

    printf ("g_slave1_proc()..\n");

    if (((rplidar_serv_port_str = getenv("RPLIDAR_SERV_PORT"))!=NULL) && 
        (rplidar_serv_port_str[0]!='\0')) {
        rplidar_serv_port = atoi(rplidar_serv_port_str);
    }
    printf("INFO: launching ZMQ publisher on port : %d\n", rplidar_serv_port);

    rc = CommZmq::instance().init(rplidar_serv_port);
    if (rc<0) {
        printf("ERROR: CommZmq::instance().init() failed\n");
        return NULL;
    }

    CommZmq::instance().taskFunction();

    return NULL;
}


void *g_slave2_proc(void*)
{
    printf ("g_slave2_proc()..\n");

    RobotDetect::instance().init();

    while (!CommZmq::instance().taskRunning()) {
        pthread_yield();
    }

#if 0 /* FIXME : DEBUG */
    RobotDetect::instance().taskFunctionFunny();
#else
    while (CommZmq::instance().taskRunning()) {
        pthread_yield();
    }
#endif

    return NULL;
}



