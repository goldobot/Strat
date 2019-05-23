#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <zmq.h>
#include <math.h>
#include <pthread.h>
#include <iostream>

#include "robot_detect.hpp"

#include "comm_zmq.hpp"

using namespace goldobot;

/* FIXME : TODO */


RobotDetect RobotDetect::s_instance;

RobotDetect& RobotDetect::instance()
{
	return s_instance;
}

RobotDetect::RobotDetect()
{
    m_stop_task = false;
    m_task_running = false;

    /* FIXME : TODO */
}

int RobotDetect::init()
{
    /* FIXME : TODO */

    return 0;
}

void RobotDetect::taskFunctionFunny()
{
    struct timespec curr_tp;
    int curr_time_ms = 0;
    int old_time_ms = 0;
    float adv_x_m[3];
    float adv_y_m[3];
    float R_adv_m = 0.2;
    double theta_adv_rad[3];

    theta_adv_rad[0] = 0.0;
    adv_x_m[0] = 1.0 + R_adv_m*cos(theta_adv_rad[0]);
    adv_y_m[0] =-1.2 + R_adv_m*sin(theta_adv_rad[0]);

    theta_adv_rad[1] = 0.0;
    adv_x_m[1] = 0.4 + R_adv_m*cos(theta_adv_rad[1]);
    adv_y_m[1] = 1.0 + R_adv_m*sin(theta_adv_rad[1]);

    theta_adv_rad[2] = 0.0;
    adv_x_m[2] = 1.0 + R_adv_m*cos(theta_adv_rad[2]);
    adv_y_m[2] = 1.0 + R_adv_m*sin(theta_adv_rad[2]);

    m_task_running = true;

    while(!m_stop_task)
    {
        clock_gettime(1, &curr_tp);

        curr_time_ms = curr_tp.tv_sec*1000 + curr_tp.tv_nsec/1000000;

        if (curr_time_ms > (old_time_ms + 100)) {
            unsigned short my_message_type;
            RobotDetectionMsg my_message;

            my_message_type = 1280; /* FIXME : TODO : use mesage_types.hpp */

            double tmp_double;

            theta_adv_rad[0] += M_PI/100.0;
            adv_x_m[0] = 1.0 + R_adv_m*cos(theta_adv_rad[0]);
            adv_y_m[0] =-0.8 + R_adv_m*sin(theta_adv_rad[0]);

            my_message.timestamp_ms = curr_time_ms;
            my_message.id = 0;
            tmp_double = 4000.0*adv_x_m[0];
            my_message.x_mm_X4 = tmp_double;
            tmp_double = 4000.0*adv_y_m[0];
            my_message.y_mm_X4 = tmp_double;
            my_message.vx_mm_sec = 0;
            my_message.vy_mm_sec = 0;
            my_message.ax_mm_sec_2 = 0;
            my_message.ay_mm_sec_2 = 0;

            CommZmq::instance().send((const char*)(&my_message_type), 2, ZMQ_SNDMORE);
            CommZmq::instance().send((const char*)(&my_message), sizeof(my_message), 0);

            theta_adv_rad[1] += 8.0*M_PI/100.0;
            adv_x_m[1] = 0.4 + R_adv_m*cos(theta_adv_rad[1]);
            adv_y_m[1] = 0.8 + R_adv_m*sin(theta_adv_rad[1]);

            my_message.timestamp_ms = curr_time_ms;
            my_message.id = 1;
            tmp_double = 4000.0*adv_x_m[1];
            my_message.x_mm_X4 = tmp_double;
            tmp_double = 4000.0*adv_y_m[1];
            my_message.y_mm_X4 = tmp_double;
            my_message.vx_mm_sec = 0;
            my_message.vy_mm_sec = 0;
            my_message.ax_mm_sec_2 = 0;
            my_message.ay_mm_sec_2 = 0;

            CommZmq::instance().send((const char*)(&my_message_type), 2, ZMQ_SNDMORE);
            CommZmq::instance().send((const char*)(&my_message), sizeof(my_message), 0);

            theta_adv_rad[2] += 16.0*M_PI/100.0;
            adv_x_m[2] = 1.0 + R_adv_m*cos(theta_adv_rad[2]);
            adv_y_m[2] = 0.8 + R_adv_m*sin(theta_adv_rad[2]);

            my_message.timestamp_ms = curr_time_ms;
            my_message.id = 2;
            tmp_double = 4000.0*adv_x_m[2];
            my_message.x_mm_X4 = tmp_double;
            tmp_double = 4000.0*adv_y_m[2];
            my_message.y_mm_X4 = tmp_double;
            my_message.vx_mm_sec = 0;
            my_message.vy_mm_sec = 0;
            my_message.ax_mm_sec_2 = 0;
            my_message.ay_mm_sec_2 = 0;

            CommZmq::instance().send((const char*)(&my_message_type), 2, ZMQ_SNDMORE);
            CommZmq::instance().send((const char*)(&my_message), sizeof(my_message), 0);

            old_time_ms = curr_time_ms;
        }

        pthread_yield();
    }

    m_task_running = false;
}
