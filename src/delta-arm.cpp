#include "delta-arm.hpp"
// #include "../include/delta-arm.hpp"

namespace DeltaArm {
    
    Motor::Motor(float start_pos) : start_pos(start_pos) {

    }

    void Motor::set_tar_pos(float pos) {}

    
    float Arm::arm_get_theta(void) {}
    float Arm::arm_get_tar_pos(float theta) {}

    void Arm::arm_set_tar_pos() {

    }
            

    /**
        * @brief Initialize hardware layer / port
        */
    void Arm::init() {}
    
    Vec3 Arm::get_cur_pos(void) {}
    
    Vec3 Arm::get_tar_pos(void) {}
    void Arm::set_tar_pos(float x, float y, float z) {}
    
    void Arm::apply(void) {}
}