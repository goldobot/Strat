#pragma once
#include <cstdint>
#include <cstddef>

namespace goldobot
{
    class CommZmq
    {
    public:
        static CommZmq& instance();

        CommZmq ();

        int init(int port_nb);

        void taskFunction();

        bool taskRunning() {return m_task_running;}

        void stopTask() {m_stop_task = true;}

        int send(const void *buf, size_t len, int flags);

        int recv(void *buf, size_t len, int flags);

    private:
        bool m_stop_task = false;
        bool m_task_running = false;

        void* m_zmq_context;
        void* m_pub_socket;
        void* m_pull_socket;

        static CommZmq s_instance;
    };

}
