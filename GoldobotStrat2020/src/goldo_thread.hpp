#pragma once
#include <cstdint>
#include <cstddef>

#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <iostream>

namespace goldobot
{
  class GoldoThread
  {
  public:
    GoldoThread()
      {
        m_stop_task = false;
        m_task_running = false;
      };

    int startProcessing()
      {
        int ret;
        ret=pthread_create(&m_thread_id, NULL, &taskFunctionWrapper, (void *)this);
        if(ret!=0) {
          return -1;
        }
        return 0;
      };

    static void *taskFunctionWrapper(void* ctxt)
      {
        GoldoThread *my_obj = (GoldoThread *)ctxt;
        /* FIXME : DEBUG */
        if ((my_obj==NULL))
        {
          printf ("DEBUG : null context for GoldoThread::taskFunctionWrapper() !\n");
          return NULL;
        }
        my_obj->taskFunction();
        return NULL;
      };

    virtual void taskFunction() = 0;

    bool taskRunning() 
      {
        return m_task_running;
      };

    void stopTask() 
      {
        m_stop_task = true;
      };

  protected:
    bool m_stop_task = false;
    bool m_task_running = false;

  private:
    pthread_t m_thread_id;
  };
}

