#pragma once
#include <cstdint>
#include <cstddef>

#include "yaml-cpp/yaml.h"
#include "yaml-cpp/eventhandler.h"

#include "strat/robot_strat_types.hpp"
#include "strat/robot_strat_base.hpp"

#include "sim/simul_types.hpp"

#ifndef ROBOT_SIM
#error "You must define ROBOT_SIM"
#endif

namespace goldobot
{

  class VirtualRobot {
  public:
    static VirtualRobot& myself();
    static VirtualRobot& partner();
    static VirtualRobot& adversary1();
    static VirtualRobot& adversary2();

    VirtualRobot();

    void set_enable(bool enable_flag) {m_enabled = enable_flag;};

    void set_autom(bool autom_flag) {m_automatic = autom_flag;};

    int read_yaml_conf(YAML::Node &yconf);

    sim_motion_state_vector_t& sv() {return m_sv;};

    /* FIXME : TODO : code duplication is bad! */
    static const unsigned int GPIO_GREEN_LED_MASK          = 0x00000001;
    static const unsigned int GPIO_START_MASK              = 0x00000002;
    static const unsigned int GPIO_EMERGENCY_STOP_MASK     = 0x00000004;
    static const unsigned int GPIO_DYNAMIXEL_DIR_MASK      = 0x00000008;
    static const unsigned int GPIO_SIDE_SELECT_MASK        = 0x00000010;
    static const unsigned int GPIO_AUTOCONFIG_MASK         = 0x00000020;

    static const unsigned int FLAG_PROPULSION_BUSY_MASK    = 0x00010000;
    static const unsigned int FLAG_PROPULSION_ERROR_MASK   = 0x00020000;

    unsigned int& gpio() {return m_gpio;};

    void sim_update(double t_inc);

    void sim_receive(const unsigned char *msg_buf, size_t msg_len);

    void sim_brutal_stop();

    void sim_compute_motion_times(double L, double V, 
                                  double A, double D, 
                                  double &t_a, double &t_i, double &t_d);

    /* FIXME : TODO */
  private:
    void on_cmd_execute_trajectory(unsigned char *msg_buf, size_t msg_len);
    void on_cmd_execute_point_to(unsigned char *msg_buf, size_t msg_len);
    void on_cmd_set_pose(unsigned char *msg_buf, size_t msg_len);
    void on_cmd_start_sequence(unsigned char *msg_buf, size_t msg_len);
    void on_cmd_propulsion_clear_error(unsigned char *msg_buf, size_t msg_len);

    void create_rotation_me(sim_vec_2d_t &orig, double orig_theta, 
                            sim_vec_2d_t &target, 
                            float speed, float accel, float deccel);
    void create_translation_me(sim_vec_2d_t &orig, 
                               sim_vec_2d_t &target, 
                               float speed, float accel, float deccel);

    bool m_enabled;

    bool m_automatic;

    /* FIXME : TODO : geometric model of the robot (with conf section) */
    double m_prop_semi_axis;

    sim_motion_state_vector_t m_sv;

    unsigned int m_gpio;

    StratTask *default_strat;

    static const int MOTION_ELEM_LIST_SZ = 10000;
    sim_motion_element_t *m_me;
    int m_me_idx; /* -1 acts as a "enable execution flag" */
    int m_me_cnt;

    unsigned char m_recv_buf[256];
    static const int RECV_BUF_SZ = sizeof(m_recv_buf);

    static VirtualRobot s_myself;
    static VirtualRobot s_partner;
    static VirtualRobot s_adversary1;
    static VirtualRobot s_adversary2;
  };

}
