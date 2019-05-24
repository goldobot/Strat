#pragma once
#include <cstdint>
#include <cstddef>

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


    class RobotDetect
    {
    public:
        static RobotDetect& instance();

        RobotDetect();

        void taskFunctionFunny();

        bool taskRunning() {return m_task_running;}

        void stopTask() {m_stop_task = true;}

        int init();

        void clearSlots();

        void processNewRplidarSample(unsigned int ts_ms, double x_mm, double y_mm);

        void updateDetection();

        void sendDetected();

        /* FIXME : TODO */

    private:
        bool m_stop_task = false;
        bool m_task_running = false;

        CurrentDetectionSlot m_detect_slot[MAX_NB_OF_DETECTION_SLOTS];

        DetectedRobot m_detect_t_0[MAX_NB_OF_DETECTED_ROBOTS]; /* detectes pendant le scan courant(t0) */
        DetectedRobot m_detect_t_1[MAX_NB_OF_DETECTED_ROBOTS]; /* detectes a t0-1*dt */
        DetectedRobot m_detect_t_2[MAX_NB_OF_DETECTED_ROBOTS]; /* detectes a t0-2*dt */

        static double dist(double x0, double y0, double x1, double y1);

        static RobotDetect s_instance;
    };

}
