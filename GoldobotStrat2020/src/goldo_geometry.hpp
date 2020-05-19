#pragma once
#include <cstdint>
#include <cstddef>
#include <math.h>

namespace goldobot
{

/**  Base types  *************************************************************/

  typedef struct _goldo_vec_2d {
    double x;
    double y;
  } goldo_vec_2d_t;

  typedef struct _goldo_segm_2d {
    goldo_vec_2d_t p1;
    goldo_vec_2d_t p2;
  } goldo_segm_2d_t;

  inline double goldo_normalize_angle(
    double theta)
  {
    int n;
    double nf;
    if (theta>M_PI)
    {
      nf = theta/(2.0*M_PI);
      n = nf;
      n++;
      return theta-(double)n*(2.0*M_PI);
    }
    else if (theta<=-M_PI)
    {
      nf = -theta/(2.0*M_PI);
      n = nf;
      n++;
      return theta+(double)n*(2.0*M_PI);
    }
    return theta;
  }

  inline double goldo_dist(
    double p2_x, double p2_y, double p1_x, double p1_y)
  {
    double dx = p2_x - p1_x;
    double dy = p2_y - p1_y;
    return sqrt (dx*dx + dy*dy);
  }

  inline double goldo_dist(
    goldo_vec_2d_t &p2, goldo_vec_2d_t &p1)
  {
    return goldo_dist(p2.x, p2.y, p1.x, p1.y);
  }

  inline double goldo_segm_dist(
    double o_x, double o_y, double p1_x, double p1_y, double p2_x, double p2_y)
  {
    double my_dist = 0.0;

    double d1 = goldo_dist(o_x, o_y, p1_x, p1_y);
    double d2 = goldo_dist(o_x, o_y, p2_x, p2_y);
    double Lsegm = goldo_dist(p2_x, p2_y, p1_x, p1_y);
    double proj1 = fabs((o_x-p1_x)*(p2_x-p1_x)+(o_y-p1_y)*(p2_y-p1_y))/Lsegm;
    double proj2 = fabs((o_x-p2_x)*(p2_x-p1_x)+(o_y-p2_y)*(p2_y-p1_y))/Lsegm;

    if ((proj1+proj2-1.0e-09f)>Lsegm)
    {
      my_dist=(proj1<proj2)?d1:d2;
    }
    else
    {
      double v1_x = p1_x-o_x;
      double v1_y = p1_y-o_y;
      double v2_x = p2_x-o_x;
      double v2_y = p2_y-o_y;
      my_dist=fabs(v1_x*v2_y-v1_y*v2_x)/Lsegm;
    }

    return my_dist;
  }

  inline double goldo_segm_dist(
    goldo_vec_2d_t &o, goldo_segm_2d_t &s)
  {
    return goldo_segm_dist(o.x, o.y, s.p1.x, s.p1.y, s.p2.x, s.p2.y);
  }

  inline double goldo_compute_theta(
    double orig_x, double orig_y, double target_x, double target_y)
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

  inline double goldo_compute_theta(
    goldo_vec_2d_t &orig, goldo_vec_2d_t &target)
  {
    return goldo_compute_theta(orig.x, orig.y, target.x, target.y);
  }

}

