/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

// unitree_sdk2 related headfile
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/robot/client/client.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>

#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <thread>
#include <lcm/lcm-cpp.hpp>
#include "state_estimator_lcmt.hpp"
#include "leg_control_data_lcmt.hpp"
#include "pd_tau_targets_lcmt.hpp"
#include "rc_command_lcmt.hpp"
#include "gripper_lcmt.hpp"

#include "utility.h"
#include "Hardware/can.h"
#include "Hardware/motor.h"
#include "App/arm_control.h"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <atomic>
#include <cmath>
#include <iostream>
#include <algorithm>

using namespace std;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_JOYSTICK "rt/wirelesscontroller"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

// No need to change: Motor calibration function provided by Unitree
uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{   
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

// Remote control key value union, extracted from unitree_sdk2, no need to change
typedef union
{
  struct
  {
    uint8_t R1 : 1;
    uint8_t L1 : 1;
    uint8_t start : 1;
    uint8_t select : 1;
    uint8_t R2 : 1;
    uint8_t L2 : 1;
    uint8_t F1 : 1;
    uint8_t F2 : 1;
    uint8_t A : 1;
    uint8_t B : 1;
    uint8_t X : 1;
    uint8_t Y : 1;
    uint8_t up : 1;
    uint8_t right : 1;
    uint8_t down : 1;
    uint8_t left : 1;
  } components;
  uint16_t value;
} xKeySwitchUnion;


class Custom
{
public:
    explicit Custom(){}
    ~Custom(){}

    void Init();
    void InitLowCmd();
    void Loop();

    void LowStateMessageHandler(const void* messages);
    void JoystickHandler(const void *message);
    void InitRobotStateClient();
    void activateService(const std::string& serviceName,int activate);

    int queryServiceStatus(const std::string& serviceName);
    void SetNominalPose();
    void LowCmdWrite();
    void ARXLowCmdWrite();


    void lcm_send();
    void lcm_receive();
    void handleActionLCM(const lcm::ReceiveBuffer *rbuf, const std::string & chan, const pd_tau_targets_lcmt * msg);
    void handleGripperLCM(const lcm::ReceiveBuffer *rbuf, const std::string & chan, const gripper_lcmt * msg);

    /*LowCmd write thread*/
    // DDS related low-level command sending thread pointer
    unitree::common::ThreadPtr LcmSendThreadPtr;
    unitree::common::ThreadPtr LcmRecevThreadPtr;
    unitree::common::ThreadPtr lowCmdWriteThreadPtr;
    unitree::common::ThreadPtr ARXLowCmdWriteThreadPtr;

    unitree_go::msg::dds_::LowState_ low_state{};
    unitree_go::msg::dds_::LowCmd_ low_cmd{};     
    unitree_go::msg::dds_::WirelessController_ joystick{};
    unitree::robot::go2::RobotStateClient rsc;

    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> joystick_suber;

    lcm::LCM lc = lcm::LCM("udpm://239.255.76.67:7314?ttl=255");

    xKeySwitchUnion key;
    int mode = 0;
    int motiontime = 0;
    float dt = 0.002; // unit [second]
    bool _firstRun;

    state_estimator_lcmt body_state_simple = {0};
    leg_control_data_lcmt joint_state = {0};
    pd_tau_targets_lcmt joint_command = {0};
    rc_command_lcmt rc_command = {0};
    gripper_lcmt gripper_command = {4.5};
    
    arx_arm ARX_ARM;
    can CAN_Handlej;
    bool stop_flag;
};

void Custom::LowStateMessageHandler(const void* message)
{
    low_state = *(unitree_go::msg::dds_::LowState_*)message;
}

void Custom::JoystickHandler(const void *message)
{
    joystick = *(unitree_go::msg::dds_::WirelessController_ *)message;
    key.value = joystick.keys();
}

void Custom::InitRobotStateClient()
{
    rsc.SetTimeout(5.0f); 
    rsc.Init();
}

int Custom::queryServiceStatus(const std::string& serviceName)
{
    std::vector<unitree::robot::go2::ServiceState> serviceStateList;
    int ret,serviceStatus;
    ret = rsc.ServiceList(serviceStateList);
    size_t i, count=serviceStateList.size();
    for (i=0; i<count; i++)
    {
        const unitree::robot::go2::ServiceState& serviceState = serviceStateList[i];
        if(serviceState.name == serviceName)
        {
            if(serviceState.status == 0)
            {
                std::cout << "name: " << serviceState.name <<" is activate"<<std::endl;
                serviceStatus = 1;
            }
            else
            {
                std::cout << "name:" << serviceState.name <<" is deactivate"<<std::endl;
                serviceStatus = 0;
            } 
        }    
    }
    return serviceStatus;
    
}

void Custom::activateService(const std::string& serviceName,int activate)
{
    rsc.ServiceSwitch(serviceName, activate);  
}


// -------------------------------------------------------------------------------
// Thread 3: unitree_sdk2 command write thread
// Purpose: Initialize low_cmd, after reasonable state machine, motors will execute neural network output
void Custom::InitLowCmd()
{
    // head in LowCmd type represents frame header,
    // this header will be used for CRC check. head, levelFlag, gpio etc. should be set to default values as shown
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    /* LowCmd type has 20 motorCmd members,
    each member's command controls one corresponding motor on Go2 robot,
    but Go2 robot only has 12 motors,
    so only first 12 are valid, remaining 8 are reserved. */
    for(int i=0; i<20; i++)
    {
        /* This command sets motorCmd member's mode variable to 0x01,
        0x01 means setting motor to servo mode.
        If users find they cannot control Go2 robot's joint motors during debugging,
        please check if this value is 0x01. */
        low_cmd.motor_cmd()[i].mode() = (0x01);   // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        low_cmd.motor_cmd()[i].kd() = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
    }
}

void Custom::SetNominalPose(){
    // After running this cpp file, not only initializes communication
    // But also initializes joint angles when robot is lying down
    // Set all motors to position mode
    for(int i = 0; i < 12; i++){
        joint_command.qd_des[i] = 0;
        joint_command.tau_ff[i] = 0;
        joint_command.kp[i] = 50.;
        joint_command.kd[i] = 1.;
    }

    joint_command.q_des[0] = -0.3;
    joint_command.q_des[1] = 1.2;
    joint_command.q_des[2] = -2.721;
    joint_command.q_des[3] = 0.3;
    joint_command.q_des[4] = 1.2;
    joint_command.q_des[5] = -2.721;
    joint_command.q_des[6] = -0.3;
    joint_command.q_des[7] = 1.2;
    joint_command.q_des[8] = -2.721;
    joint_command.q_des[9] = 0.3;
    joint_command.q_des[10] = 1.2;
    joint_command.q_des[11] = -2.721;

    std::cout<<"SET NOMINAL POSE"<<std::endl;

    // init arm
    CAN_Handlej.Send_moto_Cmd1(1, 0, 12, 0, 0, 0);
    CAN_Handlej.Send_moto_Cmd1(2, 0, 12, 0, 0, 0);usleep(200);
    CAN_Handlej.Send_moto_Cmd1(4, 0, 12, 0, 0, 0);usleep(200);
    CAN_Handlej.Send_moto_Cmd2(5, 0, 1, 0, 0, 0);usleep(200);
    CAN_Handlej.Send_moto_Cmd2(6, 0, 1, 0, 0, 0);usleep(200);
    CAN_Handlej.Send_moto_Cmd2(7, 0, 1, 0, 0, 0);usleep(200);
    CAN_Handlej.Send_moto_Cmd2(8, 0, 1, 0, 0, 0);usleep(200);
    ARX_ARM.get_joint();
    ARX_ARM.target_pos_temp[0] = ARX_ARM.current_pos[0];
    ARX_ARM.target_pos_temp[1]=ARX_ARM.current_pos[1];
    ARX_ARM.target_pos_temp[2]=ARX_ARM.current_pos[2];
    ARX_ARM.target_pos_temp[3]=ARX_ARM.current_pos[3];
    ARX_ARM.target_pos_temp[4]=ARX_ARM.current_pos[4];
    ARX_ARM.target_pos_temp[5]=ARX_ARM.current_pos[5];
    ARX_ARM.target_pos_temp[6]=ARX_ARM.current_pos[6];

    joint_command.q_arm_des[0] = 0;
    joint_command.q_arm_des[1] = 0;
    joint_command.q_arm_des[2] = 0;
    joint_command.q_arm_des[3] = 0;
    joint_command.q_arm_des[4] = 0;
    joint_command.q_arm_des[5] = 0;
    joint_command.q_arm_des[6] = 0;
    
    for(int j = 0; j<6; j++){
        ARX_ARM.target_pos[j]=joint_command.q_arm_des[j];
    }
    ARX_ARM.target_pos[6] = 4.5;
    
    for(int i=0;i < 1000;i++)
    {
        ARX_ARM.get_joint();
        ARX_ARM.update_real();
		usleep(4200);
    }


    ARX_ARM.get_joint();
    
    joint_command.q_arm_des[0]=ARX_ARM.current_pos[0];
    joint_command.q_arm_des[1]=ARX_ARM.current_pos[1];
    joint_command.q_arm_des[2]=ARX_ARM.current_pos[2];
    joint_command.q_arm_des[3]=ARX_ARM.current_pos[3];
    joint_command.q_arm_des[4]=ARX_ARM.current_pos[4];
    joint_command.q_arm_des[5]=ARX_ARM.current_pos[5];
    joint_command.q_arm_des[6]=ARX_ARM.current_pos[6];

    printf("SET ARM NOMINAL POSE");

}

void Custom::lcm_receive(){
    while(true){
        lc.handle();
    }
}


void Custom::Init(){
    _firstRun = true;
    stop_flag = false;

    InitLowCmd();
    SetNominalPose();

    lc.subscribe("pd_plustau_targets", &Custom::handleActionLCM, this);
    lc.subscribe("gripper_command", &Custom::handleGripperLCM, this);

    /*create low_cmd publisher*/
    lowcmd_publisher.reset(new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();
    /*create low_state dds subscriber*/
    lowstate_subscriber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);
    /*create joystick dds subscriber*/
    joystick_suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK));
    joystick_suber->InitChannel(std::bind(&Custom::JoystickHandler, this, std::placeholders::_1), 1);
}



double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}

void Custom::handleActionLCM(const lcm::ReceiveBuffer *rbuf, const std::string & chan, const pd_tau_targets_lcmt * msg){
    (void) rbuf;
    (void) chan;

    joint_command = *msg;
}

void Custom::handleGripperLCM(const lcm::ReceiveBuffer *rbuf, const std::string & chan, const gripper_lcmt * msg){
    (void) rbuf;
    (void) chan;

    gripper_command = *msg;
    // std::cout<< joint_command.q_des[0] << std::endl;
}



void Custom::lcm_send(){

    rc_command.left_stick[0] = joystick.lx();
    rc_command.left_stick[1] = joystick.ly();
    rc_command.right_stick[0] = joystick.rx();
    rc_command.right_stick[1] = joystick.ry();
    rc_command.right_lower_right_switch = key.components.R2;
    rc_command.right_upper_switch = key.components.R1;
    rc_command.left_lower_left_switch = key.components.L2;
    rc_command.left_upper_switch = key.components.L1;

    if(key.components.A > 0){
        mode = 0;
    } else if(key.components.B > 0){
        mode = 1;
    }else if(key.components.X > 0){
        mode = 2;
    }else if(key.components.Y > 0){
        mode = 3;
    }else if(key.components.up > 0){
        mode = 4;
    }else if(key.components.right > 0){
        mode = 5;
    }else if(key.components.down > 0){
        mode = 6;
    }else if(key.components.left > 0){
        mode = 7;
    }

    rc_command.mode = mode;


    // publish state to LCM
    for(int i = 0; i < 12; i++){
        joint_state.q[i] = low_state.motor_state()[i].q();
        joint_state.qd[i] = low_state.motor_state()[i].dq();
        joint_state.tau_est[i] = low_state.motor_state()[i].tau_est();
    }
    for(int i = 0; i < 4; i++){
        body_state_simple.quat[i] = low_state.imu_state().quaternion()[i]; 
    }
    for(int i = 0; i < 3; i++){
        body_state_simple.rpy[i] = low_state.imu_state().rpy()[i];
        body_state_simple.aBody[i] = low_state.imu_state().accelerometer()[i];
        body_state_simple.omegaBody[i] = low_state.imu_state().gyroscope()[i];
    }
    for(int i = 0; i < 4; i++){
        body_state_simple.contact_estimate[i] = low_state.foot_force()[i];
    }

    // arm current joint pose
    ARX_ARM.get_joint();
    for (int i = 0; i < 6; i++){
        joint_state.q_arm[i] = ARX_ARM.current_pos[i];
    }

    lc.publish("leg_state_estimator_data", &body_state_simple);
    lc.publish("leg_control_data", &joint_state);
    lc.publish("rc_command", &rc_command);

}



void Custom::Loop(){
    // Additional threads can implement loop function functionality

    // intervalMicrosec: 1 microsecond = 0.000001 second
    // when dt=0.002s
    // intervalMicrosec = 2000us
    /*lcm send thread*/
    LcmSendThreadPtr = unitree::common::CreateRecurrentThreadEx("lcm_send_thread", UT_CPU_ID_NONE, dt*1e6, &Custom::lcm_send, this);
    /*lcm receive thread*/
    LcmRecevThreadPtr = unitree::common::CreateRecurrentThreadEx("lcm_recev_thread", UT_CPU_ID_NONE, dt*1e6, &Custom::lcm_receive, this);
    /*low command write thread*/
    lowCmdWriteThreadPtr = unitree::common::CreateRecurrentThreadEx("dds_write_thread", UT_CPU_ID_NONE, dt*1e6, &Custom::LowCmdWrite, this);
    /*ARX low command write thread*/
    ARXLowCmdWriteThreadPtr = unitree::common::CreateRecurrentThreadEx("arx_lowcmd_write_thread", UT_CPU_ID_NONE, dt*1e6, &Custom::ARXLowCmdWrite, this);
}

void Custom::ARXLowCmdWrite(){

    stop_flag = false;
    if (!stop_flag){
        for(int i = 0; i < 6; i++){
            ARX_ARM.target_pos[i] =  joint_command.q_arm_des[i];
        }
        // ARX_ARM.update_real();
        ARX_ARM.target_pos[6] = std::min(4.5, std::max(0.0, double(gripper_command.gripper_cmd)));


        ARX_ARM.limit_joint(ARX_ARM.target_pos);

        ARX_ARM.target_pos_temp[0] = ARX_ARM.ramp(ARX_ARM.target_pos[0], ARX_ARM.target_pos_temp[0], 0.008);
        ARX_ARM.target_pos_temp[1] = ARX_ARM.ramp(ARX_ARM.target_pos[1], ARX_ARM.target_pos_temp[1], 0.008);
        ARX_ARM.target_pos_temp[2] = ARX_ARM.ramp(ARX_ARM.target_pos[2], ARX_ARM.target_pos_temp[2], 0.008);
        ARX_ARM.target_pos_temp[3] = ARX_ARM.ramp(ARX_ARM.target_pos[3], ARX_ARM.target_pos_temp[3], 0.008);
        ARX_ARM.target_pos_temp[4] = ARX_ARM.ramp(ARX_ARM.target_pos[4], ARX_ARM.target_pos_temp[4], 0.008);
        ARX_ARM.target_pos_temp[5] = ARX_ARM.ramp(ARX_ARM.target_pos[5], ARX_ARM.target_pos_temp[5], 0.008);
        ARX_ARM.target_pos_temp[6] = ARX_ARM.ramp(ARX_ARM.target_pos[6], ARX_ARM.target_pos_temp[6], 0.05);

        CAN_Handlej.Send_moto_Cmd1(1, 90, 28, ARX_ARM.target_pos_temp[0], 0, 0);usleep(200);
        CAN_Handlej.Send_moto_Cmd1(2, 90, 30, ARX_ARM.target_pos_temp[1], 0, 0);usleep(200);
        CAN_Handlej.Send_moto_Cmd1(4, 90, 30, ARX_ARM.target_pos_temp[2], 0, 0);usleep(200);
        CAN_Handlej.Send_moto_Cmd2(5, 30, 1.2, ARX_ARM.target_pos_temp[3], 0, 0);usleep(200);
        CAN_Handlej.Send_moto_Cmd2(6, 25, 1, ARX_ARM.target_pos_temp[4], 0, 0);usleep(200);
        CAN_Handlej.Send_moto_Cmd2(7, 20, 1,   ARX_ARM.target_pos_temp[5], 0, 0);usleep(200);
        CAN_Handlej.Send_moto_Cmd2(8, 5, 1,   ARX_ARM.target_pos_temp[6], 0, 0);usleep(200);

    }
    else if (stop_flag) {
        
        ARX_ARM.target_pos_temp[0] = ARX_ARM.ramp(ARX_ARM.target_pos[0], ARX_ARM.target_pos_temp[0],0.001);
        ARX_ARM.target_pos_temp[1] = ARX_ARM.ramp(ARX_ARM.target_pos[1], ARX_ARM.target_pos_temp[1],0.001);
        ARX_ARM.target_pos_temp[2] = ARX_ARM.ramp(ARX_ARM.target_pos[2], ARX_ARM.target_pos_temp[2],0.001);
        ARX_ARM.target_pos_temp[3] = ARX_ARM.ramp(ARX_ARM.target_pos[3], ARX_ARM.target_pos_temp[3],0.001);
        ARX_ARM.target_pos_temp[4] = ARX_ARM.ramp(ARX_ARM.target_pos[4], ARX_ARM.target_pos_temp[4],0.001);
        ARX_ARM.target_pos_temp[5] = ARX_ARM.ramp(ARX_ARM.target_pos[5], ARX_ARM.target_pos_temp[5],0.001);
        ARX_ARM.target_pos_temp[6] = ARX_ARM.ramp(ARX_ARM.target_pos[6], ARX_ARM.target_pos_temp[6],0.001);

        CAN_Handlej.Send_moto_Cmd1(1, 150, 12, ARX_ARM.target_pos_temp[0], 0, 0);usleep(200);
        CAN_Handlej.Send_moto_Cmd1(2, 150, 12, ARX_ARM.target_pos_temp[1], 0, 0);usleep(200);
        CAN_Handlej.Send_moto_Cmd1(4, 150, 12, ARX_ARM.target_pos_temp[2], 0, 0);usleep(200);
        CAN_Handlej.Send_moto_Cmd2(5, 30, 0.8, ARX_ARM.target_pos_temp[3], 0, 0);usleep(200);
        CAN_Handlej.Send_moto_Cmd2(6, 25, 0.8, ARX_ARM.target_pos_temp[4], 0, 0);usleep(200);
        CAN_Handlej.Send_moto_Cmd2(7, 10, 1,   ARX_ARM.target_pos_temp[5], 0, 0);usleep(200);
        CAN_Handlej.Send_moto_Cmd2(8, 10, 1,   ARX_ARM.target_pos_temp[6], 0, 0);usleep(200);
    }

    usleep(4200);

}


void Custom::LowCmdWrite(){
    motiontime++;

    if(_firstRun && joint_state.q[0] != 0){
        for(int i = 0; i < 12; i++){
            joint_command.q_des[i] = joint_state.q[i];
            // Initialize L2+B to prevent accidental damping activation
            key.components.Y = 0;
            key.components.A = 0;
            key.components.B = 0;
            key.components.L2 = 0;
        }
        _firstRun = false;
    }

    if ( std::abs(low_state.imu_state().rpy()[0]) > 1.0 || std::abs(low_state.imu_state().rpy()[1]) > 1.0 || ((int)key.components.B==1 && (int)key.components.L2==1))
    {       
        for (int i = 0; i < 12; i++){
            // Enter damping mode
            low_cmd.motor_cmd()[i].q() = 0;
            low_cmd.motor_cmd()[i].dq() = 0;
            low_cmd.motor_cmd()[i].kp() = 0;
            low_cmd.motor_cmd()[i].kd() = 5;
            low_cmd.motor_cmd()[i].tau() = 0;
        }  
        std::cout << "======= Switched to Damping Mode, and the thread is sleeping ========"<<std::endl;
        sleep(1.5);

        // TODO: Need to add protection for the robotic arm
        while (true)
        {   
            if (((int)key.components.B==1 && (int)key.components.L2==1) ) {
                std::cout << "======= [L2+B] is pressed again, the script is about to exit========" <<std::endl;
                exit(0);
            } else if (((int)key.components.A==1 && (int)key.components.L2==1) ){
                rsc.ServiceSwitch("sport_mode", 1);
                std::cout << "======= activate sport_mode service and exit========" <<std::endl;
                sleep(0.5);
                exit(0);
            } else{   
                if (((int)key.components.Y==1 && (int)key.components.L2==1) ){
                    std::cout << "=======  Switch to Walk These Ways ========"<<std::endl;
                    std::cout<<"Communication is set up successfully" << std::endl;
                    std::cout<<"LCM <<<------------>>> Unitree SDK2" << std::endl;
                    std::cout<<"------------------------------------" << std::endl;
                    std::cout<<"------------------------------------" << std::endl;
                    std::cout<<"Press L2+B if any unexpected error occurs" << std::endl;
                    break;
                    
                }else{
                    std::cout << "======= Press [L2+B] again to exit ========"<<std::endl;
                    std::cout << "======= Press [L2+Y] again to switch to WTW ========"<<std::endl;
                    std::cout << "======= Press [L2+A] again to activate sport_mode service========"<<std::endl;
                    sleep(0.01);
                }
            }
        }
    }
    else{
        for(int i = 0; i < 12; i++){
            low_cmd.motor_cmd()[i].q() = joint_command.q_des[i];
            low_cmd.motor_cmd()[i].dq() = joint_command.qd_des[i];
            low_cmd.motor_cmd()[i].kp() = joint_command.kp[i];
            low_cmd.motor_cmd()[i].kd() = joint_command.kd[i];
            low_cmd.motor_cmd()[i].tau() = joint_command.tau_ff[i];
        }




    }


    // This code block first calculates CRC check code.
    // The last line calls lowcmd_publisher's Write() function to send control commands to Go2 robot.
    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
    lowcmd_publisher->Write(low_cmd);
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }

    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Caution: The scripts is about to shutdown Unitree sport_mode Service." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // Initialize with local network interface (PC or Jetson Orin)
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);


    Custom custom;

    custom.InitRobotStateClient();
    if(custom.queryServiceStatus("sport_mode"))
    {
        std::cout<<"Trying to deactivate the service: " << "sport_mode" << std::endl;
        custom.activateService("sport_mode",0);
        sleep(0.5);
        if(!custom.queryServiceStatus("sport_mode")){
            std::cout<<"Trying to deactivate the service: " << "sport_mode" << std::endl;
        }
    } else{
        std::cout <<"sportd_mode is already deactivated now" << std::endl
                  <<"next step is setting up communication" << std::endl
                  << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
    }


    custom.Init();

    std::cout<<"Communicatino is set up successfully" << std::endl;
    std::cout<<"LCM <<<------------>>> Unitree SDK2" << std::endl;
    std::cout<<"------------------------------------" << std::endl;
    std::cout<<"------------------------------------" << std::endl;
    std::cout<<"Press L2+B if any unexpected error occurs" << std::endl;

    custom.Loop();

    while (true)
    {
        sleep(10);
    }

    return 0;
}
