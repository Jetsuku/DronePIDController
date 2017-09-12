#ifndef _FLTCTRL_H_
#define _FLTCTRL_H_

#include "IMU_sense.h"
#include "Arduino.h"
#include <Servo.h>

//ESC Arduino Nano Pin
#define esc_lh_f_pin 9
#define esc_rh_f_pin 6
#define esc_lh_r_pin 10
#define esc_rh_r_pin 5

//UNCOMMENT TO DISPLAY DEBUG
//#define DEBUG;

class FLT_CNTRL
{
  
  public:
  
    //Constructor
    FLT_CNTRL();

    //Public Methods
    void CMD(int throt_cmd, int pitch_cmd, int yaw_cmd, int roll_cmd, bool cmd_received[4]);
    void BEGIN();
    void CAL_ESC(int throt_cmd, int pitch_cmd);
    void CAL_REC(int pitch_cmd, int yaw_cmd, int roll_cmd);

    private:

    Servo LH_F_ESC;
    Servo RH_F_ESC;
    Servo LH_R_ESC;
    Servo RH_R_ESC;

    bool motor_prime;
    bool prime_check[2];
    bool signal_lost;
    bool cal_check;

    int roll_input;
    int pitch_input;
    int yaw_input;
    int pitch_cmd_old;
    int roll_cmd_old;
    int yaw_cmd_old;
    int count_no_signal;
    int yaw_cmd_bias;
    int pitch_cmd_bias;
    int roll_cmd_bias;
    unsigned long time_no_signal;
    
    double yaw;
    double pitch;
    double roll;
    double Pbody;
    double Qbody;
    double Rbody;
    double roll_integral;
    double pitch_integral;
    double yaw_rate_integral;
    double pilot_pitch;
    double pilot_yaw_rate;
    double pilot_roll;
    double pilot_pitch_old;
    double pilot_yaw_rate_old;
    double pilot_roll_old;
    double flat_throt_cmd_old;

    //Private Methods
    void PRIME_MOTOR(int throt_cmd);
    void PID_CONTROLLER (int yaw_cmd, int pitch_cmd, int roll_cmd, double pilot_yaw_rate, double pilot_pitch, double pilot_roll, double yaw, double pitch, double roll, double Pbody, double Qbody, double Rbody);
    void IMU_INPUTS();
    int PILOT_CMD(int throt_cmd, int yaw_cmd, int pitch_cmd, int roll_cmd, bool cmd_received[4]);
       
};



#endif /* _FLTCTRL_H_ */
