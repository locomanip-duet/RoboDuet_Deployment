#include "utility.h"
#include "Hardware/can.h"
#include "Hardware/motor.h"
#include "App/arm_control.h"
#include "App/keyboard.h"
//#include "App/arm_control.cpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <atomic>
#include <cmath>
#include <iostream>

int main(int argc, char **argv)
{

    arx_arm ARX_ARM;
    can CAN_Handlej;
    arx5_keyboard ARX_KEYBOARD;
    std::thread keyThread(&arx5_keyboard::detectKeyPress, &ARX_KEYBOARD);

    ARX_ARM.target_pos[0]=0;
    ARX_ARM.target_pos[1]=0;
    ARX_ARM.target_pos[2]=0;
    ARX_ARM.target_pos[3]=0;
    ARX_ARM.target_pos[4]=0;
    ARX_ARM.target_pos[5]=0;
    ARX_ARM.target_pos[6]=0;
    sleep(1);

    while(1)
    { 
        char key = ARX_KEYBOARD.keyPress.load();
        if(key=='q'){
            break;
        }

        ARX_ARM.target_pos[0]=0;
        ARX_ARM.target_pos[1]=0.8;
        ARX_ARM.target_pos[2]=0.8;
        ARX_ARM.target_pos[3]=0;
        ARX_ARM.target_pos[4]=0;
        ARX_ARM.target_pos[5]=0;
        ARX_ARM.target_pos[6]=0;

        ARX_ARM.get_joint();
        ARX_ARM.update_real();
		usleep(4200);
    }

    ARX_ARM.target_pos[0]=0;
    ARX_ARM.target_pos[1]=0;
    ARX_ARM.target_pos[2]=0;
    ARX_ARM.target_pos[3]=0;
    ARX_ARM.target_pos[4]=0;
    ARX_ARM.target_pos[5]=0;
    ARX_ARM.target_pos[6]=0;

    for(int i=0; i<2000; i++){
        ARX_ARM.get_joint();
        ARX_ARM.update_real();
		usleep(4200);
    }

    return 0;
}
