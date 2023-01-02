#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include "yaml-cpp/yaml.h"
#include "yaml-cpp/eventhandler.h"

#include "goldo_conf.hpp"


using namespace goldobot;


char GoldoConf::conf_viewer_addr_str_def[]     = "192.168.0.241";
char GoldoConf::conf_rplidar_dev_str_def[]     = "/dev/rplidar";
char GoldoConf::conf_nucleo_uart_dev_str_def[] = "/dev/odometry";
char GoldoConf::conf_strat_file_str_def[]      = "strat.yaml";
char GoldoConf::conf_strat_file_pos_str_def[]  = "";
char GoldoConf::conf_strat_file_neg_str_def[]  = "";
char GoldoConf::conf_simul_file_str_def[]      = "simul.yaml";



GoldoConf GoldoConf::s_instance;

GoldoConf& GoldoConf::instance()
{
  return s_instance;
}

GoldoConf::GoldoConf()
{
  memset (&m_conf_file_name, 0, sizeof(m_conf_file_name));

  memset(&m_c, 0, sizeof(m_c));

  set_default();
}

void GoldoConf::set_default()
{
  strncpy(m_c.conf_viewer_addr_str, conf_viewer_addr_str_def, 
          sizeof(m_c.conf_viewer_addr_str)-1);
  m_c.conf_theta_correction_deg = conf_theta_correction_deg_def;
  m_c.conf_rho_correction_factor = conf_rho_correction_factor_def;
  strncpy(m_c.conf_rplidar_dev_str, conf_rplidar_dev_str_def, 
          sizeof(m_c.conf_rplidar_dev_str)-1);
  m_c.conf_rplidar_baudrate = conf_rplidar_baudrate_def;
  m_c.conf_rplidar_plot_lifetime_ms = conf_rplidar_plot_lifetime_ms_def;
  m_c.conf_rplidar_send_plot_enabled = conf_rplidar_send_plot_enabled_def;
  m_c.conf_rplidar_debug_plot_enabled = conf_rplidar_debug_plot_enabled_def;
  m_c.conf_rplidar_plot_nz = 0;
  strncpy(m_c.conf_nucleo_uart_dev_str, conf_nucleo_uart_dev_str_def, 
          sizeof(m_c.conf_nucleo_uart_dev_str)-1);
  m_c.conf_nucleo_uart_baudrate = conf_nucleo_uart_baudrate_def;
  m_c.conf_zmq_port = conf_zmq_port_def;
  m_c.conf_n_obstacles = conf_n_obstacles_def;
  strncpy(m_c.conf_strat_file_str, conf_strat_file_str_def, 
          sizeof(m_c.conf_strat_file_str)-1);
  strncpy(m_c.conf_strat_file_pos_str, conf_strat_file_pos_str_def, 
          sizeof(m_c.conf_strat_file_pos_str)-1);
  strncpy(m_c.conf_strat_file_neg_str, conf_strat_file_neg_str_def, 
          sizeof(m_c.conf_strat_file_neg_str)-1);
  strncpy(m_c.conf_simul_file_str, conf_simul_file_str_def, 
          sizeof(m_c.conf_simul_file_str)-1);
  m_c.conf_dbg_log_enabled = conf_dbg_log_enabled_def;
  m_c.conf_calib_lidar_ns = 2;
  m_c.conf_calib_lidar_sample[0] = {0.0, 0.0};
  m_c.conf_calib_lidar_sample[1] = {2995.00000, 2712.73492};
}

int GoldoConf::init(const char *conf_fname)
{
  strncpy(m_conf_file_name, conf_fname, sizeof(m_conf_file_name)-1);

  parse_yaml_conf(conf_fname);

  return 0;
}

int GoldoConf::parse_yaml_conf(const char * yaml_fname)
{
  int ret = 0;
  std::ifstream fin;
  const char *my_str = NULL;

  try 
  {
    fin.open(yaml_fname);

    YAML::Node yconf = YAML::Load(fin);
    YAML::Node conf_node;

    conf_node = yconf["environment"]["conf_viewer_addr_str"];
    if (conf_node) 
    {
      my_str = (const char *) conf_node.as<std::string>().c_str();
      strncpy(m_c.conf_viewer_addr_str, my_str, 
              sizeof(m_c.conf_viewer_addr_str)-1);
    }

    conf_node = yconf["environment"]["conf_theta_correction_deg"];
    if (conf_node) 
    {
      my_str = (const char *) conf_node.as<std::string>().c_str();
      m_c.conf_theta_correction_deg = strtod(my_str, NULL);
    }

    conf_node = yconf["environment"]["conf_rho_correction_factor"];
    if (conf_node) 
    {
      my_str = (const char *) conf_node.as<std::string>().c_str();
      m_c.conf_rho_correction_factor = strtod(my_str, NULL);
    }

    conf_node = yconf["environment"]["conf_rplidar_plot_lifetime_ms"];
    if (conf_node) 
    {
      my_str = (const char *) conf_node.as<std::string>().c_str();
      m_c.conf_rplidar_plot_lifetime_ms = strtoul(my_str, NULL, 10);
    }

    conf_node = yconf["environment"]["conf_rplidar_dev_str"];
    if (conf_node) 
    {
      my_str = (const char *) conf_node.as<std::string>().c_str();
      strncpy(m_c.conf_rplidar_dev_str, my_str, 
              sizeof(m_c.conf_rplidar_dev_str)-1);
    }

    conf_node = yconf["environment"]["conf_rplidar_baudrate"];
    if (conf_node) 
    {
      my_str = (const char *) conf_node.as<std::string>().c_str();
      m_c.conf_rplidar_baudrate = strtoul(my_str, NULL, 10);
    }

    conf_node = yconf["environment"]["conf_nucleo_uart_dev_str"];
    if (conf_node) 
    {
      my_str = (const char *) conf_node.as<std::string>().c_str();
      strncpy(m_c.conf_nucleo_uart_dev_str, my_str, 
              sizeof(m_c.conf_nucleo_uart_dev_str)-1);
    }

    conf_node = yconf["environment"]["conf_nucleo_uart_baudrate"];
    if (conf_node) 
    {
      my_str = (const char *) conf_node.as<std::string>().c_str();
      m_c.conf_nucleo_uart_baudrate = strtoul(my_str, NULL, 10);
    }

    conf_node = yconf["environment"]["conf_zmq_port"];
    if (conf_node) 
    {
      my_str = (const char *) conf_node.as<std::string>().c_str();
      m_c.conf_zmq_port = strtoul(my_str, NULL, 10);
    }

    conf_node = yconf["environment"]["conf_n_obstacles"];
    if (conf_node) 
    {
      my_str = (const char *) conf_node.as<std::string>().c_str();
      m_c.conf_n_obstacles = strtoul(my_str, NULL, 10);
    }

    conf_node = yconf["environment"]["conf_strat_file_str"];
    if (conf_node) 
    {
      my_str = (const char *) conf_node.as<std::string>().c_str();
      strncpy(m_c.conf_strat_file_str, my_str, 
              sizeof(m_c.conf_strat_file_str)-1);
    }

    conf_node = yconf["environment"]["conf_strat_file_pos_str"];
    if (conf_node) 
    {
      my_str = (const char *) conf_node.as<std::string>().c_str();
      strncpy(m_c.conf_strat_file_pos_str, my_str, 
              sizeof(m_c.conf_strat_file_pos_str)-1);
    }

    conf_node = yconf["environment"]["conf_strat_file_neg_str"];
    if (conf_node) 
    {
      my_str = (const char *) conf_node.as<std::string>().c_str();
      strncpy(m_c.conf_strat_file_neg_str, my_str, 
              sizeof(m_c.conf_strat_file_neg_str)-1);
    }

    conf_node = yconf["environment"]["conf_simul_file_str"];
    if (conf_node) 
    {
      my_str = (const char *) conf_node.as<std::string>().c_str();
      strncpy(m_c.conf_simul_file_str, my_str, 
              sizeof(m_c.conf_simul_file_str)-1);
    }

    conf_node = yconf["environment"]["conf_dbg_log"];
    if (conf_node) 
    {
      my_str = (const char *) conf_node.as<std::string>().c_str();
      if (strncmp(my_str,"enabled",7)==0)
      {
        m_c.conf_dbg_log_enabled = true;
      }
    }

    conf_node = yconf["environment"]["conf_rplidar_send_plot"];
    if (conf_node) 
    {
      my_str = (const char *) conf_node.as<std::string>().c_str();
      if (strncmp(my_str,"enabled",7)==0)
      {
        m_c.conf_rplidar_send_plot_enabled = true;
      }
    }

    conf_node = yconf["environment"]["conf_rplidar_debug_plot"];
    if (conf_node) 
    {
      my_str = (const char *) conf_node.as<std::string>().c_str();
      if (strncmp(my_str,"enabled",7)==0)
      {
        m_c.conf_rplidar_debug_plot_enabled = true;
      }
    }

    conf_node = yconf["environment"]["conf_calib_lidar"];
    if (conf_node) 
    {
      m_c.conf_calib_lidar_ns = conf_node.size();
      for (_u32 j=0; j<m_c.conf_calib_lidar_ns; j++)
      {
        my_str = conf_node[j][0].as<std::string>().c_str();
        m_c.conf_calib_lidar_sample[j].real_d = strtof(my_str, NULL);
        my_str = conf_node[j][1].as<std::string>().c_str();
        m_c.conf_calib_lidar_sample[j].meas_d = strtof(my_str, NULL);
      }
    }

    conf_node = yconf["environment"]["conf_rplidar_plot_zone"];
    if (conf_node) 
    {
      m_c.conf_rplidar_plot_nz = conf_node.size();
      for (_u32 j=0; j<m_c.conf_rplidar_plot_nz; j++)
      {
        my_str = conf_node[j][0].as<std::string>().c_str();
        m_c.conf_rplidar_plot_zone[j].x_min_mm = strtof(my_str, NULL);
        my_str = conf_node[j][1].as<std::string>().c_str();
        m_c.conf_rplidar_plot_zone[j].y_min_mm = strtof(my_str, NULL);
        my_str = conf_node[j][2].as<std::string>().c_str();
        m_c.conf_rplidar_plot_zone[j].x_max_mm = strtof(my_str, NULL);
        my_str = conf_node[j][3].as<std::string>().c_str();
        m_c.conf_rplidar_plot_zone[j].y_max_mm = strtof(my_str, NULL);
      }
    }

    ret = 0;
  } 
  catch(const YAML::Exception& e)
  {
    std::cerr << e.what() << "\n";
    ret = -1;
  }

  return ret;
}

void GoldoConf::display_conf()
{
  printf ("  conf_viewer_addr_str          = %s\n", 
             m_c.conf_viewer_addr_str);
  printf ("  conf_theta_correction_deg     = %f\n", 
             m_c.conf_theta_correction_deg);
  printf ("  conf_rho_correction_factor    = %f\n", 
             m_c.conf_rho_correction_factor);
  printf ("  conf_rplidar_plot_lifetime_ms = %d\n", 
             m_c.conf_rplidar_plot_lifetime_ms);
  printf ("  conf_rplidar_dev_str          = %s\n", 
             m_c.conf_rplidar_dev_str);
  printf ("  conf_rplidar_baudrate         = %d\n", 
             m_c.conf_rplidar_baudrate);
  printf ("  conf_rplidar_send_plot        = %s\n", 
             m_c.conf_rplidar_send_plot_enabled?"enabled":"disabled");
  printf ("  conf_rplidar_debug_plot       = %s\n", 
             m_c.conf_rplidar_debug_plot_enabled?"enabled":"disabled");
  printf ("  conf_rplidar_plot_zones:\n");
  for (_u32 i=0; i<m_c.conf_rplidar_plot_nz; i++)
  {
    printf ("    - [ %6.1f, %6.1f, %6.1f, %6.1f]\n",
            m_c.conf_rplidar_plot_zone[i].x_min_mm,
            m_c.conf_rplidar_plot_zone[i].y_min_mm,
            m_c.conf_rplidar_plot_zone[i].x_max_mm,
            m_c.conf_rplidar_plot_zone[i].y_max_mm);
  }
  printf ("  conf_nucleo_uart_dev_str      = %s\n", 
             m_c.conf_nucleo_uart_dev_str);
  printf ("  conf_nucleo_uart_baudrate     = %d\n", 
             m_c.conf_nucleo_uart_baudrate);
  printf ("  conf_zmq_port                 = %d\n", 
             m_c.conf_zmq_port);
  printf ("  conf_n_obstacles              = %d\n", 
             m_c.conf_n_obstacles);
  printf ("  conf_strat_file_str           = %s\n", 
             m_c.conf_strat_file_str);
  printf ("  conf_strat_file_pos_str       = %s\n", 
             m_c.conf_strat_file_pos_str);
  printf ("  conf_strat_file_neg_str       = %s\n", 
             m_c.conf_strat_file_neg_str);
  printf ("  conf_simul_file_str           = %s\n", 
             m_c.conf_simul_file_str);
  printf ("  conf_dbg_log                  = %s\n", 
             m_c.conf_dbg_log_enabled?"enabled":"disabled");
  printf ("  conf_calib_lidar:\n");
  for (_u32 i=0; i<m_c.conf_calib_lidar_ns; i++)
  {
    printf ("    - [ %10.5f, %10.5f]\n",
            m_c.conf_calib_lidar_sample[i].real_d,
            m_c.conf_calib_lidar_sample[i].meas_d);
  }
}

