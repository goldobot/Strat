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


RobotDetect RobotDetect::s_instance;

RobotDetect& RobotDetect::instance()
{
	return s_instance;
}

RobotDetect::RobotDetect()
{
  m_cur_ts_ms = 0;
}

int RobotDetect::init()
{
  m_cur_ts_ms = 0;

  for (int i=0; i<MAX_NB_OF_DETECTED_ROBOTS; i++)
  {
    m_detect_t_0[i].nb_rplidar_samples = 0;
    m_detect_t_0[i].timestamp_ms       = 0;
    m_detect_t_0[i].id                 = i;
    m_detect_t_0[i].x_mm               = 0.0;
    m_detect_t_0[i].y_mm               = 0.0;
    m_detect_t_0[i].vx_mm_sec          = 0.0;
    m_detect_t_0[i].vy_mm_sec          = 0.0;
    m_detect_t_0[i].ax_mm_sec_2        = 0.0;
    m_detect_t_0[i].ay_mm_sec_2        = 0.0;

    m_detect_t_1[i].nb_rplidar_samples = 0;
    m_detect_t_1[i].timestamp_ms       = 0;
    m_detect_t_1[i].id                 = i;
    m_detect_t_1[i].x_mm               = 0.0;
    m_detect_t_1[i].y_mm               = 0.0;
    m_detect_t_1[i].vx_mm_sec          = 0.0;
    m_detect_t_1[i].vy_mm_sec          = 0.0;
    m_detect_t_1[i].ax_mm_sec_2        = 0.0;
    m_detect_t_1[i].ay_mm_sec_2        = 0.0;

    m_detect_t_2[i].nb_rplidar_samples = 0;
    m_detect_t_2[i].timestamp_ms       = 0;
    m_detect_t_2[i].id                 = i;
    m_detect_t_2[i].x_mm               = 0.0;
    m_detect_t_2[i].y_mm               = 0.0;
    m_detect_t_2[i].vx_mm_sec          = 0.0;
    m_detect_t_2[i].vy_mm_sec          = 0.0;
    m_detect_t_2[i].ax_mm_sec_2        = 0.0;
    m_detect_t_2[i].ay_mm_sec_2        = 0.0;
  }

  return 0;
}

/* FIXME : DEBUG */
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

  //m_task_running = true;

  //while(!m_stop_task)
  while(1)
  {
    clock_gettime(1, &curr_tp);

    curr_time_ms = curr_tp.tv_sec*1000 + curr_tp.tv_nsec/1000000;

    if (curr_time_ms > (old_time_ms + 100)) 
    {
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
      my_message.detect_quality = 100;

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
      my_message.detect_quality = 100;

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
      my_message.detect_quality = 100;

      CommZmq::instance().send((const char*)(&my_message_type), 2, ZMQ_SNDMORE);
      CommZmq::instance().send((const char*)(&my_message), sizeof(my_message), 0);

      old_time_ms = curr_time_ms;
    }

    pthread_yield();
  }

  //m_task_running = false;
}


void RobotDetect::clearSlots()
{
  for (int i=0; i<MAX_NB_OF_DETECTION_SLOTS; i++)
  {
    m_detect_slot[i].nb_rplidar_samples = 0;
    m_detect_slot[i].timestamp_ms = 0;
    m_detect_slot[i].x_mm = 0.0;
    m_detect_slot[i].y_mm = 0.0;
  }
}


void RobotDetect::processNewRplidarSample(unsigned int ts_ms, double x_mm, double y_mm)
{
  //printf ("  new(%f,%f)\n", x_mm, y_mm);
  m_cur_ts_ms = ts_ms;

  for (int i=0; i<MAX_NB_OF_DETECTION_SLOTS; i++)
  {
    if (m_detect_slot[i].nb_rplidar_samples == 0)
    {
      m_detect_slot[i].nb_rplidar_samples = 1;
      m_detect_slot[i].timestamp_ms = ts_ms;
      m_detect_slot[i].x_mm = x_mm;
      m_detect_slot[i].y_mm = y_mm;
      return;
    }

    if (dist(m_detect_slot[i].x_mm, m_detect_slot[i].y_mm, x_mm, y_mm) < OBSTACLE_SIZE_MM)
    {
      int n = m_detect_slot[i].nb_rplidar_samples;
      m_detect_slot[i].nb_rplidar_samples++;
      if (m_detect_slot[i].timestamp_ms < ts_ms) m_detect_slot[i].timestamp_ms = ts_ms;
      m_detect_slot[i].x_mm = (x_mm + n*m_detect_slot[i].x_mm)/(n+1);
      m_detect_slot[i].y_mm = (y_mm + n*m_detect_slot[i].y_mm)/(n+1);
      return;
    }
  }
}


void RobotDetect::updateDetection()
{
  int best_samples=0;
  int best_pos=0;
  int second_samples=0;
  int second_pos=0;
  int third_samples=0;
  int third_pos=0;

  for (int i=0; i<MAX_NB_OF_DETECTION_SLOTS; i++)
  {
    if (m_detect_slot[i].nb_rplidar_samples > best_samples)
    {
      third_samples = second_samples;
      third_pos = second_pos;

      second_samples = best_samples;
      second_pos = best_pos;

      best_samples = m_detect_slot[i].nb_rplidar_samples;
      best_pos = i;
    }
    else if (m_detect_slot[i].nb_rplidar_samples > second_samples)
    {
      third_samples = second_samples;
      third_pos = second_pos;

      second_samples = m_detect_slot[i].nb_rplidar_samples;
      second_pos = i;
    }
    else if (m_detect_slot[i].nb_rplidar_samples > third_samples)
    {
      third_samples = m_detect_slot[i].nb_rplidar_samples;
      third_pos = i;
    }
  }

  m_detect_candidate[0].nb_rplidar_samples = m_detect_slot[best_pos].nb_rplidar_samples;
  m_detect_candidate[0].timestamp_ms       = m_detect_slot[best_pos].timestamp_ms;
  m_detect_candidate[0].id                 = 0xffffffff;
  m_detect_candidate[0].x_mm               = m_detect_slot[best_pos].x_mm;
  m_detect_candidate[0].y_mm               = m_detect_slot[best_pos].y_mm;

  m_detect_candidate[1].nb_rplidar_samples = m_detect_slot[second_pos].nb_rplidar_samples;
  m_detect_candidate[1].timestamp_ms       = m_detect_slot[second_pos].timestamp_ms;
  m_detect_candidate[1].id                 = 0xffffffff;
  m_detect_candidate[1].x_mm               = m_detect_slot[second_pos].x_mm;
  m_detect_candidate[1].y_mm               = m_detect_slot[second_pos].y_mm;

  m_detect_candidate[2].nb_rplidar_samples = m_detect_slot[third_pos].nb_rplidar_samples;
  m_detect_candidate[2].timestamp_ms       = m_detect_slot[third_pos].timestamp_ms;
  m_detect_candidate[2].id                 = 0xffffffff;
  m_detect_candidate[2].x_mm               = m_detect_slot[third_pos].x_mm;
  m_detect_candidate[2].y_mm               = m_detect_slot[third_pos].y_mm;

#ifdef MULTITRACKING
  for (int i=0; i<MAX_NB_OF_DETECTED_ROBOTS; i++)
  {
    m_detect_t_2[i].nb_rplidar_samples = m_detect_t_1[i].nb_rplidar_samples;
    m_detect_t_2[i].timestamp_ms       = m_detect_t_1[i].timestamp_ms;
    m_detect_t_2[i].id                 = m_detect_t_1[i].id;
    m_detect_t_2[i].x_mm               = m_detect_t_1[i].x_mm;
    m_detect_t_2[i].y_mm               = m_detect_t_2[i].y_mm;
    m_detect_t_2[i].vx_mm_sec          = m_detect_t_2[i].vx_mm_sec;
    m_detect_t_2[i].vy_mm_sec          = m_detect_t_2[i].vy_mm_sec;
    m_detect_t_2[i].ax_mm_sec_2        = m_detect_t_2[i].ax_mm_sec_2;
    m_detect_t_2[i].ay_mm_sec_2        = m_detect_t_2[i].ay_mm_sec_2;

    m_detect_t_1[i].nb_rplidar_samples = m_detect_t_0[i].nb_rplidar_samples;
    m_detect_t_1[i].timestamp_ms       = m_detect_t_0[i].timestamp_ms;
    m_detect_t_1[i].id                 = m_detect_t_0[i].id;
    m_detect_t_1[i].x_mm               = m_detect_t_0[i].x_mm;
    m_detect_t_1[i].y_mm               = m_detect_t_0[i].y_mm;
    m_detect_t_1[i].vx_mm_sec          = m_detect_t_0[i].vx_mm_sec;
    m_detect_t_1[i].vy_mm_sec          = m_detect_t_0[i].vy_mm_sec;
    m_detect_t_1[i].ax_mm_sec_2        = m_detect_t_0[i].ax_mm_sec_2;
    m_detect_t_1[i].ay_mm_sec_2        = m_detect_t_0[i].ay_mm_sec_2;
  }

  for (int i=0; i<MAX_NB_OF_DETECTED_ROBOTS; i++)
  {
    m_detect_t_0[i].id                 = 0xffffffff;
    if ((m_detect_t_0[i].nb_rplidar_samples>0))
    {
      for (int j=0; j<MAX_NB_OF_DETECTED_ROBOTS; j++)
      {
        if ((m_detect_candidate[j].nb_rplidar_samples>0) && dist(m_detect_t_0[i],m_detect_candidate[j])<100.0)
        {
          m_detect_t_0[i].nb_rplidar_samples = m_detect_candidate[j].nb_rplidar_samples;
          m_detect_t_0[i].timestamp_ms       = m_detect_candidate[j].timestamp_ms;
          m_detect_t_0[i].id                 = i;
          m_detect_t_0[i].x_mm               = m_detect_candidate[j].x_mm;
          m_detect_t_0[i].y_mm               = m_detect_candidate[j].y_mm;

          m_detect_candidate[j].id           = i;
        }
      }
    }
    else
    {
      for (int j=0; j<MAX_NB_OF_DETECTED_ROBOTS; j++)
      {
        if ((m_detect_candidate[j].nb_rplidar_samples>0) && (m_detect_candidate[j].id==0xffffffff))
        {
          m_detect_t_0[i].nb_rplidar_samples = m_detect_candidate[j].nb_rplidar_samples;
          m_detect_t_0[i].timestamp_ms       = m_detect_candidate[j].timestamp_ms;
          m_detect_t_0[i].id                 = i;
          m_detect_t_0[i].x_mm               = m_detect_candidate[j].x_mm;
          m_detect_t_0[i].y_mm               = m_detect_candidate[j].y_mm;

          m_detect_candidate[j].id           = i;
        }
      }
    }
  }

  for (int i=0; i<MAX_NB_OF_DETECTED_ROBOTS; i++)
  {
    double dt_sec = (m_detect_t_0[i].timestamp_ms - m_detect_t_1[i].timestamp_ms)/1000.0;

    m_detect_t_0[i].id                 = i;
    m_detect_t_0[i].vx_mm_sec          = (m_detect_t_0[i].x_mm - m_detect_t_1[i].x_mm)/dt_sec;
    m_detect_t_0[i].vy_mm_sec          = (m_detect_t_0[i].y_mm - m_detect_t_1[i].y_mm)/dt_sec;
    m_detect_t_0[i].ax_mm_sec_2        = (m_detect_t_0[i].vx_mm_sec - m_detect_t_1[i].vx_mm_sec)/dt_sec;
    m_detect_t_0[i].ay_mm_sec_2        = (m_detect_t_0[i].vy_mm_sec - m_detect_t_1[i].vy_mm_sec)/dt_sec;
  }

#else

  m_detect_t_0[0].nb_rplidar_samples = m_detect_slot[best_pos].nb_rplidar_samples;
  m_detect_t_0[0].timestamp_ms       = m_detect_slot[best_pos].timestamp_ms;
  m_detect_t_0[0].id                 = 0;
  m_detect_t_0[0].x_mm               = m_detect_slot[best_pos].x_mm;
  m_detect_t_0[0].y_mm               = m_detect_slot[best_pos].y_mm;

  m_detect_t_0[1].nb_rplidar_samples = m_detect_slot[second_pos].nb_rplidar_samples;
  m_detect_t_0[1].timestamp_ms       = m_detect_slot[second_pos].timestamp_ms;
  m_detect_t_0[1].id                 = 1;
  m_detect_t_0[1].x_mm               = m_detect_slot[second_pos].x_mm;
  m_detect_t_0[1].y_mm               = m_detect_slot[second_pos].y_mm;

  m_detect_t_0[2].nb_rplidar_samples = m_detect_slot[third_pos].nb_rplidar_samples;
  m_detect_t_0[2].timestamp_ms       = m_detect_slot[third_pos].timestamp_ms;
  m_detect_t_0[2].id                 = 2;
  m_detect_t_0[2].x_mm               = m_detect_slot[third_pos].x_mm;
  m_detect_t_0[2].y_mm               = m_detect_slot[third_pos].y_mm;

#endif

}


void RobotDetect::sendDetected()
{
  unsigned short my_message_type;
  RobotDetectionMsg my_message;

  my_message_type = 1280; /* FIXME : TODO : use mesage_types.hpp */

  for (int i=0; i<MAX_NB_OF_DETECTED_ROBOTS; i++)
  {
    if (m_detect_t_0[1].nb_rplidar_samples>0)
    {
      my_message.timestamp_ms   = m_detect_t_0[i].timestamp_ms;
      my_message.id             = m_detect_t_0[i].id;
      my_message.x_mm_X4        = m_detect_t_0[i].x_mm * 4.0;
      my_message.y_mm_X4        = m_detect_t_0[i].y_mm * 4.0;
      my_message.vx_mm_sec      = m_detect_t_0[i].vx_mm_sec;
      my_message.vy_mm_sec      = m_detect_t_0[i].vy_mm_sec;
      my_message.ax_mm_sec_2    = m_detect_t_0[i].ax_mm_sec_2;
      my_message.ay_mm_sec_2    = m_detect_t_0[i].ay_mm_sec_2;
      my_message.detect_quality = m_detect_t_0[i].nb_rplidar_samples;
    }
    else
    {
      my_message.timestamp_ms   = 0;
      my_message.id             = i;
      my_message.x_mm_X4        = -4000;
      my_message.y_mm_X4        = 0;
      my_message.vx_mm_sec      = 0;
      my_message.vy_mm_sec      = 0;
      my_message.ax_mm_sec_2    = 0;
      my_message.ay_mm_sec_2    = 0;
      my_message.detect_quality = 0;
    }

    CommZmq::instance().send((const char*)(&my_message_type), 2, ZMQ_SNDMORE);
    CommZmq::instance().send((const char*)(&my_message), sizeof(my_message), 0);
  }
}


double RobotDetect::dist(double x0, double y0, double x1, double y1)
{
  return sqrt((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1));
}


double RobotDetect::dist(DetectedRobot &R0, DetectedRobot &R1)
{
  return sqrt((R0.x_mm-R1.x_mm)*(R0.x_mm-R1.x_mm) + (R0.y_mm-R1.y_mm)*(R0.y_mm-R1.y_mm));
}


