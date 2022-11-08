#ifndef LINEAR_SYSTEM_H
#define LINEAR_SYSTEM_H

#include <stdint.h>
#include "arm_math.h"

typedef struct LinearSystem {
    uint16_t states;
    uint16_t inputs;
    uint16_t outputs;

    arm_matrix_instance_f32 *m_A;
    arm_matrix_instance_f32 *m_B;
    arm_matrix_instance_f32 *m_C;
    arm_matrix_instance_f32 *m_D;

} LinearSystem;

void init_linear_system(arm_matrix_instance_f32 *A, 
                        arm_matrix_instance_f32 *B, 
                        arm_matrix_instance_f32 *C, 
                        arm_matrix_instance_f32 *D);

#endif /* LINEAR_SYSTEM_H */