#pragma once
#include <cstdint>
#include <cstddef>

//#include "goldo_thread.hpp"
#include "detect/robot_detection_info.hpp"


namespace goldobot
{

#define MAX_NB_OF_DETECTION_SLOTS 400
#define MAX_NB_OF_DETECTED_ROBOTS 3
#define OBSTACLE_SIZE_MM 80.0 

  typedef struct _current_detection_slot {
    int nb_rplidar_samples;
    unsigned int timestamp_ms;
    double x_mm;
    double y_mm;
  } current_detection_slot_t;


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

    detected_robot_info_t& detected_robot(int _obst_idx);

  private:
    unsigned int m_cur_ts_ms = 0;

    current_detection_slot_t m_detect_slot[MAX_NB_OF_DETECTION_SLOTS];

    detected_robot_info_t m_detect_candidate[MAX_NB_OF_DETECTED_ROBOTS];

    detected_robot_info_t m_detect_t_0[MAX_NB_OF_DETECTED_ROBOTS]; /* detectes pendant le scan courant(t0) */
    detected_robot_info_t m_detect_t_1[MAX_NB_OF_DETECTED_ROBOTS]; /* detectes a t0-1*dt */
    detected_robot_info_t m_detect_t_2[MAX_NB_OF_DETECTED_ROBOTS]; /* detectes a t0-2*dt */
    bool m_detect_lock; /* FIXME : TODO : improve synchronisation */

    detected_robot_info_t m_detect_export[MAX_NB_OF_DETECTED_ROBOTS]; /* copie de m_detect_t_* pour visibilite depuis classes externes */


    static double dist(double x0, double y0, double x1, double y1);

    static double dist(detected_robot_info_t &R0, detected_robot_info_t &R1);

    static LidarDetect s_instance;
  };

}