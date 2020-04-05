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

    int init(int port_nb);

    virtual void taskFunction();

    int send(const void *buf, size_t len, int flags);

    int recv(void *buf, size_t len, int flags);

  private:
    void* m_zmq_context;
    void* m_pub_socket;
    void* m_pull_socket;

    static CommZmq s_instance;
  };

}
