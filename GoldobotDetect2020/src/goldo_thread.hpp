#pragma once
#ifdef WIN32
#include <windows.h>
#endif

#include <typeinfo>
#include <cstdint>
#include <cstddef>

#include <sys/time.h>
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
        strncpy(m_thread_name,"GoldoThread",sizeof(m_thread_name));
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

        printf (" %s worker thread started\n", my_obj->m_thread_name);

#if 0 /* FIXME : DEBUG */
        my_obj->m_task_running = true;

        while(!my_obj->m_stop_task)
        {
#ifndef WIN32
          usleep(1000);
#else
          Sleep(1);
#endif

#ifndef WIN32
          pthread_yield();
#else
          sched_yield();
#endif
        } /* while(!my_obj->m_stop_task) */

        my_obj->m_task_running = false;
#else
        my_obj->taskFunction();
#endif

        printf (" %s worker thread stopped\n", my_obj->m_thread_name);

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
    char m_thread_name[64];
    bool m_stop_task = false;
    bool m_task_running = false;

  private:
    pthread_t m_thread_id;
  };
}

