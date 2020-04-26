#pragma once
#include <cstdint>
#include <cstddef>

//#include "goldo_thread.hpp"


namespace goldobot
{
  typedef struct _RobotDetectionMsg {
    unsigned int timestamp_ms;
    unsigned int id;
    short int x_mm_X4;
    short int y_mm_X4;
    short int vx_mm_sec;
    short int vy_mm_sec;
    short int ax_mm_sec_2;
    short int ay_mm_sec_2;
    unsigned int detect_quality;
  } RobotDetectionMsg;


#define MAX_NB_OF_DETECTION_SLOTS 400
#define MAX_NB_OF_DETECTED_ROBOTS 3
#define OBSTACLE_SIZE_MM 80.0 

  typedef struct _CurrentDetectionSlot {
    int nb_rplidar_samples;
    unsigned int timestamp_ms;
    double x_mm;
    double y_mm;
  } CurrentDetectionSlot;

  typedef struct _DetectedRobot {
    int nb_rplidar_samples;
    unsigned int timestamp_ms;
    unsigned int id;
    double x_mm;
    double y_mm;
    double vx_mm_sec;
    double vy_mm_sec;
    double ax_mm_sec_2;
    double ay_mm_sec_2;
  } DetectedRobot;


  class LidarDetect /*: public GoldoThread*/
  {
  public:
    static LidarDetect& instance();

    LidarDetect();

    int init();

    /* FIXME : DEBUG */
    void taskFunctionFunny();

    void clearSlots();

    void processNewLidarSample(unsigned int ts_ms, double x_mm, double y_mm);

    void updateDetection();

    void sendDetected();

    DetectedRobot& get_detected_mob_obst(int _obst_idx);

  private:
    unsigned int m_cur_ts_ms = 0;

    CurrentDetectionSlot m_detect_slot[MAX_NB_OF_DETECTION_SLOTS];

    DetectedRobot m_detect_candidate[MAX_NB_OF_DETECTED_ROBOTS];

    DetectedRobot m_detect_t_0[MAX_NB_OF_DETECTED_ROBOTS]; /* detectes pendant le scan courant(t0) */
    DetectedRobot m_detect_t_1[MAX_NB_OF_DETECTED_ROBOTS]; /* detectes a t0-1*dt */
    DetectedRobot m_detect_t_2[MAX_NB_OF_DETECTED_ROBOTS]; /* detectes a t0-2*dt */
    bool m_detect_lock; /* FIXME : TODO : improve synchronisation */

    DetectedRobot m_detect_export[MAX_NB_OF_DETECTED_ROBOTS]; /* copie de m_detect_t_* pour visibilite depuis classes externes */


    static double dist(double x0, double y0, double x1, double y1);

    static double dist(DetectedRobot &R0, DetectedRobot &R1);

    static LidarDetect s_instance;
  };

}
