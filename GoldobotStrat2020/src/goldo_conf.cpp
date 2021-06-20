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
  m_c.conf_rplidar_plot_lifetime_ms = conf_rplidar_plot_lifetime_ms_def;
  strncpy(m_c.conf_rplidar_dev_str, conf_rplidar_dev_str_def, 
          sizeof(m_c.conf_rplidar_dev_str)-1);
  m_c.conf_rplidar_baudrate = conf_rplidar_baudrate_def;
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
}

