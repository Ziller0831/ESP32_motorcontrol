#ifndef _setup_H_
#define _setup_H_
#endif

#include <Arduino.h>

class diff_car {
    public:

    private:
        /* Left_motor control pin define */
        int PWM1 = 18;
        int INA1 = 19;
        int INB1 = 21;
        /* Right_motor control pin define */
        int PWM2 = 25;
        int INA2 = 33;
        int INB2 = 32;
        /* Encoder pin define */
        int L_ENC_A = 35;
        int L_ENC_B = 36;
        int R_
};