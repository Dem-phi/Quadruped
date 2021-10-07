//
// Created by demphi on 2021/10/1.
//

#ifndef _COMMON_PARAMETER_
#define _COMMON_PARAMETER_

#include "reference/common_include.h"

/**
  * @param beta -> duty cycle
  * @param phi  -> phase position of Right Hind
  */
namespace quad{
/*    enum beta{
        walk_beta,   // 0.75
        trot_beta,   // 0.5
        pace_beta,   // 0.5
        gallop_beta  // 0.5
    }beta;*/

    Vec4f beta = {0.75, 0.5, 0.5, 0.5};

/*    enum phi{
        walk_phi,    // 0.25,
        trot_phi,    // 0,
        pace_phi,    // 0.5,
        gallop_phi   // 0.5,
    };*/
    Vec4f phi = {0.25, 0.0, 0.5, 0.5};
    float DEFAULT_SHOULD_ANGLE = 10.0;


}


#endif 
