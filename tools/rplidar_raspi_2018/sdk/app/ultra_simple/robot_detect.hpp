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
    } RobotDetectionMsg;

    class RobotDetect
    {
    public:
        static RobotDetect& instance();

        RobotDetect();

        void taskFunctionFunny();

        bool taskRunning() {return m_task_running;}

        void stopTask() {m_stop_task = true;}

        int init();

        /* FIXME : TODO */

    private:
        bool m_stop_task = false;
        bool m_task_running = false;

        /* FIXME : TODO */

        static RobotDetect s_instance;
    };

}
