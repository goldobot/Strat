#pragma once
#include <cstdint>
#include <cstddef>
#include <math.h>

#include "goldo_geometry.hpp"

/* FIXME : TODO : move all these to a geometry module */

namespace goldobot
{

/**  Motion state vector  ****************************************************/

  typedef struct _sim_motion_state_vector {
    goldo_vec_2d_t p;
    double theta;
    goldo_vec_2d_t v;
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

}

