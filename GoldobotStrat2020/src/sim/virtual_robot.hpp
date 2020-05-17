#pragma once
#include <cstdint>
#include <cstddef>

#include "yaml-cpp/yaml.h"
#include "yaml-cpp/eventhandler.h"

#include "strat/robot_strat_types.hpp"
#include "strat/robot_strat_base.hpp"

#include "sim/simul_types.hpp"

#ifdef ROS
#include "robot_simulator.hpp"
#include "goldo/odometry/simple_odometry.hpp"
#include "goldo/control/propulsion_controller.hpp"
#endif

#ifndef ROBOT_SIM
#error "You must define ROBOT_SIM"
#endif

namespace goldobot
{

  class VirtualRobot {
  public:
    VirtualRobot();

    void set_enable(bool enable_flag) {m_enabled = enable_flag;};

    bool enabled() {return m_enabled;};

    void set_autom(bool autom_flag) {m_automatic = autom_flag;};

    virtual int read_yaml_conf(YAML::Node &yconf);

    void create_me_list_from_strat();

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

    virtual void sim_update(double t_inc);

    void sim_receive(const unsigned char *msg_buf, size_t msg_len);

    virtual void sim_brutal_stop();

    void sim_compute_motion_times(double L, double V, 
                                  double A, double D, 
                                  double &t_a, double &t_i, double &t_d);

    /* FIXME : TODO */
  protected:
    virtual void on_cmd_execute_trajectory(unsigned char *_buf, size_t _len);
    virtual void on_cmd_execute_point_to(unsigned char *_buf, size_t _len);
    virtual void on_cmd_set_pose(unsigned char *_buf, size_t _len);
    virtual void on_cmd_start_sequence(unsigned char *_buf, size_t _len);
    virtual void on_cmd_propulsion_clear_error(unsigned char *_buf,size_t _len);

    void create_rotation_me(sim_vec_2d_t &orig, double orig_theta, 
                            sim_vec_2d_t &target, 
                            float speed, float accel, float deccel);
    void create_translation_me(sim_vec_2d_t &orig, bool forward, 
                               sim_vec_2d_t &target, 
                               float speed, float accel, float deccel);

    bool m_enabled;

    bool m_automatic;

    /* FIXME : TODO : geometric model of the robot (with conf section) */
    double m_prop_semi_axis;

    double m_dbg_duration;

    sim_motion_state_vector_t m_sv;

    unsigned int m_gpio;

    StratTask *m_default_strat;

    static const int MOTION_ELEM_LIST_SZ = 10000;
    sim_motion_element_t *m_me;
    int m_me_idx; /* -1 acts as a "enable execution flag" */
    int m_me_cnt;

    unsigned char m_recv_buf[256];
    static const int RECV_BUF_SZ = sizeof(m_recv_buf);

  private:
  };

  class VirtualRobotROSImport : public VirtualRobot {
  public:
    VirtualRobotROSImport();

    virtual void sim_update(double t_inc);

    virtual int read_yaml_conf(YAML::Node &yconf);

  protected:
    virtual void on_cmd_execute_trajectory(unsigned char *_buf, size_t _len);
    virtual void on_cmd_execute_point_to(unsigned char *_buf, size_t _len);
    virtual void on_cmd_set_pose(unsigned char *_buf, size_t _len);
    virtual void on_cmd_propulsion_clear_error(unsigned char *_buf,size_t _len);

#ifdef ROS
    static constexpr double ROS_PROPULSION_STEP_PERIOD = 0.001;
    double m_ros_update_propulsion_time;

    goldo::SimpleOdometry m_ros_odometry;
    goldo::PropulsionController *m_ros_propulsion_controller;
    goldo::RobotSimulator m_ros_robot_simulator;
#endif

    bool m_ros_emul;

  private:
  };

  typedef class VirtualRobotROSImport MyselfVirtualRobotType;
  //typedef class VirtualRobot MyselfVirtualRobotType;
  typedef class VirtualRobot PartnerVirtualRobotType;
  typedef class VirtualRobot Adversary1VirtualRobotType;
  typedef class VirtualRobot Adversary2VirtualRobotType;

  class VirtualRobots {
  public:
    static MyselfVirtualRobotType& myself() {return s_myself;}
    static PartnerVirtualRobotType& partner() {return s_partner;}
    static Adversary1VirtualRobotType& adversary1() {return s_adversary1;}
    static Adversary2VirtualRobotType& adversary2() {return s_adversary2;}

  private:
    static MyselfVirtualRobotType s_myself;
    static PartnerVirtualRobotType s_partner;
    static Adversary1VirtualRobotType s_adversary1;
    static Adversary2VirtualRobotType s_adversary2;
  };

}
