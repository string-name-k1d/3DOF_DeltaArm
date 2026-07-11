#include "FashionStar_UartServo.h"
// #include "../motor_driver/fashionstart-uart-servo//include//FashionStar_UartServo.h"

namespace DeltaArm{
    using namespace fsuservo;
    
    typedef struct Vec3_Struct Vec3;
    // typedef struct Motor_Struct Motor;
    
    struct Vec3_Struct {
        float x;
        float y;
        float z;
    };

    struct Motor_Struct {
        const float start_pos;
        float cur_pos;
        float tar_pos;
    };

    class Motor: FSUS_Servo {
        public:
            const float start_pos;

            Motor(float start_pos);
            void set_tar_pos(float pos);
    };

    class Arm {
        private:
            Motor motors[3];

            Vec3 tar_pos;

            float arm_get_theta();
            float arm_get_tar_pos(float theta);
            
            void arm_set_tar_pos();
        
        public:
            /**
             * @brief Initialize hardware layer / port
             */
            void init();
            
            Vec3 get_cur_pos(void);
            
            Vec3 get_tar_pos(void);
            void set_tar_pos(float x, float y, float z);
            
            void apply(void);
    };
}