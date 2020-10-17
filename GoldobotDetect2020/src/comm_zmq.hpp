#pragma once
#include <cstdint>
#include <cstddef>

#include "goldo_thread.hpp"

namespace goldobot
{
  class CommZmq : public GoldoThread
  {
  public:
    static CommZmq& instance();

    CommZmq ();

    int init(int port_nb, int comm_uart_port_nb=3001);

    virtual void taskFunction();

    int send(const void *buf, size_t len, int flags);

    int recv(void *buf, size_t len, int flags);

  private:
    void* m_zmq_context;
    void* m_pub_socket;
    void* m_pull_socket;
    void* m_comm_uart_pub_socket;
    void* m_comm_uart_pull_socket;

    void dbg_dump_msg(
      FILE *dbg_log_fd, const char *prefix, unsigned char *buff, size_t len);

    static CommZmq s_instance;
  };

}
