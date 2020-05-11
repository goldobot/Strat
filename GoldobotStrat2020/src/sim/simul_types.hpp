#pragma once
#include <cstdint>
#include <cstddef>
#include <math.h>

namespace goldobot
{

/**  Motion state vector  ****************************************************/

  typedef struct _sim_vec_2d {
    double x;
    double y;
  } sim_vec_2d_t;

  typedef struct _sim_motion_state_vector {
    sim_vec_2d_t p;
    double theta;
    sim_vec_2d_t v;
    double v_theta;
  } sim_motion_state_vector_t;

/**  Motion element  *********************************************************/

  typedef enum _sim_motion_element_type {
    SM_ELEM_DYN,
    SM_ELEM_STOP,
    SM_ELEM_MARKER,
  } sim_motion_element_type_t;

  typedef struct _sim_motion_element {
    sim_motion_element_type_t type;
    union {
      struct {
        double duration;
        float lin_accel;
        float ang_accel;
      } dyn;
      int mark;
    } u;
  } sim_motion_element_t;

  inline double sim_normalize_angle(double theta)
  {
    int n;
    double nf;
    if (theta>M_PI)
    {
      nf = theta/(2.0*M_PI);
      n = nf;
      return theta-(double)n*(2.0*M_PI);
    }
    else if (theta<=-M_PI)
    {
      nf = -theta/(2.0*M_PI);
      n = nf;
      return theta+(double)n*(2.0*M_PI);
    }
    return theta;
  }

  inline double sim_dist(double p2_x, double p2_y, double p1_x, double p1_y)
  {
    double dx = p2_x - p1_x;
    double dy = p2_y - p1_y;
    return sqrt (dx*dx + dy*dy);
  }

  inline double sim_dist(sim_vec_2d_t &p2, sim_vec_2d_t &p1)
  {
    return sim_dist(p2.x, p2.y, p1.x, p1.y);
  }

  inline double sim_compute_theta(double orig_x, double orig_y, double target_x, double target_y)
  {
    double dx = target_x - orig_x;
    double dy = target_y - orig_y;
    double theta = 0.0;

    if (dx==0.0)
    {
      if (dy>0.0)
        theta = M_PI/2;
      else if (dy<0.0)
        theta = -M_PI/2;
    }
    else
    {
      theta = atan(dy/fabs(dx));
      if (dx<0.0)
      {
        if (theta<0.0)
          theta = -M_PI -theta;
        else
          theta =  M_PI -theta;
      }
    }

    return theta;
  }

  inline double sim_compute_theta(sim_vec_2d_t &orig, sim_vec_2d_t &target)
  {
    return sim_compute_theta(orig.x, orig.y, target.x, target.y);
  }

}

