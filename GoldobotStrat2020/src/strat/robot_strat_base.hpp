#pragma once
#include <cstdint>
#include <cstddef>

#include "yaml-cpp/yaml.h"
#include "yaml-cpp/eventhandler.h"

#include "astar/astar.h"
#include "goldo_thread.hpp"
#include "robot_strat_types.hpp"

namespace goldobot
{

/**  Playground  *************************************************************/

  class StratPlayground {
  public:
    StratPlayground();

    void init();

    void init_pattern(unsigned char *_patt, 
                      int _patt_sz_cm, int _obst_r_cm, 
                      unsigned char _code);

    void put_pattern(int x_cm, int y_cm, unsigned char*_patt, int _patt_sz_cm);

    void put_stat_rect_obst(int x_min_mm, int x_max_mm,
                            int y_min_mm, int y_max_mm);

    void put_mob_point_obst(int x_mm, int y_mm);

    void erase_mob_obst();

    void feed_astar(AStar & _astar);

    void create_playground_ppm();

    void dump_playground_ppm(char *ppm_fname);

    void send_playground_ppm();


    static const int Y_OFFSET_CM     = 150;
    static const int X_MIN_CM        =   0;
    static const int X_MAX_CM        = 200;
    static const int X_SZ_CM         = X_MAX_CM-X_MIN_CM;
    static const int Y_MIN_CM        =-150;
    static const int Y_MAX_CM        = 150;
    static const int Y_SZ_CM         = Y_MAX_CM-Y_MIN_CM;
    static const int S_OBST_R_CM     =  14;
    static const int M_OBST_R_CM     =  34;
    static const int S_PATT_SZ_CM    = 2*S_OBST_R_CM;
    static const int M_PATT_SZ_CM    = 2*M_OBST_R_CM;

    static const unsigned char NO_OBST  = 255;
    static const unsigned char S_OBST   =   0;
    static const unsigned char M_OBST   =   1;
    static const unsigned char S_EXCL   =  64;
    static const unsigned char M_EXCL   =  65;
    static const unsigned char PATH     = 128;
    static const unsigned char OLD_PATH = 129;
    static const unsigned char PATH_WP  = 160;

    unsigned char m_playground[X_SZ_CM * Y_SZ_CM];
    unsigned char m_stat_playground[X_SZ_CM * Y_SZ_CM];
    unsigned char m_stat_pattern[S_PATT_SZ_CM * S_PATT_SZ_CM];
    unsigned char m_mob_pattern[M_PATT_SZ_CM * M_PATT_SZ_CM];

    int m_ppm_sz;
    unsigned char m_ppm_buff[0x40000];
  };


/**  Strat tasks  *************************************************************/

  class StratTask {
  public:
    StratTask();

    int read_yaml_conf (YAML::Node &yconf);

    void dbg_dump_task();

    /* FIXME : TODO */

    char m_task_name[64];
    int m_curr_act_idx;
    int m_n_actions;
    strat_action_t *m_action_list[128];
    unsigned char m_action_buf[16384];
    unsigned char m_emergency_action_buf[4096];
    int m_priority;
    bool m_started;
    bool m_completed;
    strat_way_point_t m_init_pos_wp;
    strat_way_point_t m_init_point_to_wp;
    strat_way_point_t m_current_action_final_wp;
    int m_min_init_goto_duration_ms;
    int m_max_init_goto_duration_ms;
    float m_required_pos_accuracy_mm;
    float m_required_ang_accuracy_cos;
  };

}

