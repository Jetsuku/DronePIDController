#include "flight_controller.h"
#include <math.h>

    
    static unsigned long max_recover_time = 5000;       //5 seconds Recover Time to Land
    static int max_count_no_signal = 100;               //100 No Signal Measurments Allowed Before Recover Override
    static int recover_motor_cmd = 1350;
    static int deadband = 25;                           //32 original

    //Controller Gains
    static double Kp_pitch = 2.0;     //3.5
    static double Kp_roll = 2.0;      //3.5
    static double Kd_yaw_rate = 10.0;
    static double Kd_pitch = 1.05;     //0.8
    static double Kd_roll = 1.05;      //0.8
    static double Ki_pitch = 0.5;     //0.5
    static double Ki_roll = 0.5;      //0.5
    static double Ki_yaw_rate = 3.0;

    //IMU Bias & Sample Rate
    static double sample_rate = 1.0/300.0;    //(MPU6050 ~300 Measurements per sec)
    static double pitch_sense_bias = 4.25;   //-0.63 
    static double roll_sense_bias = 1.59;     //2.11
  
FLT_CNTRL::FLT_CNTRL()
: pitch_cmd_old(1500), yaw_cmd_old(1500), roll_cmd_old(1500), roll_input(0), pitch_input(0), yaw_input(0), 
pitch_integral(0), roll_integral(0), yaw_rate_integral(0), yaw(0.0), pitch(0.0), roll(0.0), Pbody(0.0), Qbody(0.0), Rbody(0.0),
signal_lost(false), prime_check{false}, count_no_signal(0), time_no_signal(0), pilot_pitch_old(0), pilot_yaw_rate_old(0), 
pilot_roll_old(0), flat_throt_cmd_old(1000), cal_check(false)
{

}

void FLT_CNTRL::CAL_REC(int pitch_cmd, int yaw_cmd, int roll_cmd){

            pitch_cmd_bias = 1500;
            yaw_cmd_bias = 1500;
            roll_cmd_bias = 1500;
         
}     

void FLT_CNTRL::CAL_ESC(int throt_cmd, int pitch_cmd){

    //CALIBRATE ESC//
      if ((throt_cmd > 1750) && (cal_check == false)){          //Check Throttle Stick Moved To Top       
         cal_check = true;
      }

      
      if (cal_check == true){
        while (millis() < 1500){
          //Inputs for Throttle Highpoint - ESC Calibration
          LH_F_ESC.writeMicroseconds(throt_cmd);
          RH_F_ESC.writeMicroseconds(throt_cmd);
          LH_R_ESC.writeMicroseconds(throt_cmd);
          RH_R_ESC.writeMicroseconds(throt_cmd);
        }
      }
}

void FLT_CNTRL::PRIME_MOTOR(int throt_cmd){

    //PRIME MOTOR SEQUENCE//
    if ((throt_cmd > 1750) && (motor_prime == false) && (cal_check == false)){          //Check Throttle Stick Moved To Top       
      prime_check[0] = true;
    }
    else if ((throt_cmd < 1150) && (prime_check[0] == true)){   //Check Throttle Stick Moved To Bottom
      prime_check[1] = true;
    }
    else if ((throt_cmd > 1150) && (prime_check[1] == true)){   //Start Motors, Reset Prime Checks
      motor_prime = true;
      prime_check[0] = false;
      prime_check[1] = false;
    }

}

void FLT_CNTRL::PID_CONTROLLER (int yaw_cmd, int pitch_cmd, int roll_cmd, double pilot_yaw_rate, double pilot_pitch, double pilot_roll, double yaw, double pitch, double roll, double Pbody, double Qbody, double Rbody) {

    //CALCULATE ERROR INTEGRAL w/o WINDUP
    
    if ((abs(pitch_cmd_old - pitch_cmd) < deadband) && (abs(yaw_cmd_old - yaw_cmd) < deadband) && (abs(roll_cmd_old - roll_cmd) < deadband)) {                                                                               //Control Inside Ticking Range, Calculate Integral                               
        roll_integral = roll_integral + (pilot_roll - roll)*sample_rate;      //Total Roll Error
        pitch_integral = pitch_integral + (pilot_pitch - pitch)*sample_rate;  //Total Pitch Error
        yaw_rate_integral = yaw_rate_integral + (pilot_yaw_rate - Rbody)*sample_rate;     //Total Yaw Rate Error
      }
    else {                                                                                                                                                                                                      //Control Outside Ticking Range, Calculate Integral
        roll_integral = 0.0;
        pitch_integral = 0.0;
        yaw_rate_integral = 0.0;
        pitch_cmd_old = pitch_cmd;
        yaw_cmd_old = yaw_cmd;
        roll_cmd_old = roll_cmd;       
      }

    //PID CONTROLLER
    pitch_input = (int)(-Kp_pitch*(pilot_pitch - pitch) + Kd_pitch*(Pbody) - Ki_pitch*(pitch_integral));
    yaw_input = (int)(-Kd_yaw_rate*(pilot_yaw_rate - Rbody)) - Ki_yaw_rate*(yaw_rate_integral);  
    roll_input = (int)(-Kp_roll*(pilot_roll - roll) + Kd_roll*(Qbody) - Ki_roll*(roll_integral));
}

void FLT_CNTRL::IMU_INPUTS() {

    //Quad Body Axis vs. Measured Axis Compensation
    yaw = ypr[0]*180/3.14159;             //Yaw = Yaw Measured
    pitch = (-1*ypr[2]*180/3.14159) - pitch_sense_bias;        //Pitch = -1 x Roll Measured
    roll =  (ypr[1]*180/3.14159) - roll_sense_bias;           //Roll = Pitch Measured
    Pbody = ((double) -gyro[0]);       
    Qbody = ((double) -gyro[1]);    
    Rbody = ((double) -gyro[2]);     
  
}

int FLT_CNTRL::PILOT_CMD(int throt_cmd, int yaw_cmd, int pitch_cmd, int roll_cmd, bool cmd_received[3]) {

    int flat_throt_cmd;
    
    //Flatten Throttle Command
       
    //Ease Out Throttle Curve a(THROT_CMD-1000)^2 + b(THROT_CMD-1000) = ESC_CMD, a = -0.001, b = 2
    //Ease In Throttle Curve a(THROT_CMD-1000)^2 + b(THROT_CMD-1000) = ESC_CMD, a = 0.001, b = 0
    //flat_throt_cmd = (int)(0.001*((double)(throt_cmd-1000))*((double)(throt_cmd-1000)) + 0.0*((double)(throt_cmd-1000))) + 1000;
    
    if (throt_cmd < 1250){
      flat_throt_cmd = (int)(1.2*(double)(throt_cmd-1000) + 1000.0);
    }
    else if (throt_cmd > 1950){
      flat_throt_cmd = (int)(6.0*(double)(throt_cmd-1000) - 4000.0);
    }
    else {
      flat_throt_cmd = (int)(0.571*(double)(throt_cmd-1000) + 1157.3);
    }
            
    //Pilot Command
    pilot_pitch = -(45.0 / 500.0)*((double)pitch_cmd - (double)pitch_cmd_bias); //Max +/-45 deg Pitch
    pilot_yaw_rate = -(120.0 / 500.0)*((double)yaw_cmd - (double)yaw_cmd_bias); //Max +/-120 deg/sec Yaw Rate
    pilot_roll = -(45.0 / 500.0)*((double)roll_cmd - (double)roll_cmd_bias);  //Max +/-45 deg Roll Roll Rate

    if (abs(pitch_cmd - 1500) < deadband){
      pilot_pitch = 0.0;
      }
    if (abs(yaw_cmd - 1500) < deadband){
      pilot_yaw_rate = 0.0;
      }
    if (abs(roll_cmd - 1500) < deadband){
      pilot_roll = 0.0;
      }
        
    if ((cmd_received[0] & true) || (cmd_received[1] & true) || (cmd_received[2] & true)){
      count_no_signal = 0;
      time_no_signal = 0;
      pilot_pitch_old = pilot_pitch;
      pilot_yaw_rate_old = pilot_yaw_rate;
      pilot_roll_old = pilot_roll;
      flat_throt_cmd_old = flat_throt_cmd;
      signal_lost = false;
           
      }
    else if (count_no_signal <= max_count_no_signal){
      pilot_pitch = pilot_pitch_old;
      pilot_yaw_rate = pilot_yaw_rate_old;
      pilot_roll = pilot_roll_old;
      flat_throt_cmd = flat_throt_cmd_old;
      signal_lost = true;

        if (count_no_signal == max_count_no_signal) {
          time_no_signal = millis();
        }

      count_no_signal = count_no_signal + 1;
      
      }
    else if ((signal_lost == true) && ((millis() - time_no_signal) < max_recover_time) && (motor_prime == true)){
        pilot_pitch = 0.0;
        pilot_yaw_rate = 0.0;
        pilot_roll = 0.0;
        flat_throt_cmd = recover_motor_cmd;

      }
    else {
        motor_prime = false;
      }
        
    return flat_throt_cmd;
  
}

void FLT_CNTRL::CMD(int throt_cmd, int pitch_cmd, int yaw_cmd, int roll_cmd, bool cmd_received[3]){
    
    //PRIME MOTOR//
      PRIME_MOTOR(throt_cmd);

    //GET IMPU INPUTS
      IMU_INPUTS();

    //PILOT COMMAND
      int flat_throt_cmd = PILOT_CMD(throt_cmd, yaw_cmd, pitch_cmd, roll_cmd, cmd_received);
      
    //PID CONTROLLER
      PID_CONTROLLER (yaw_cmd, pitch_cmd, roll_cmd, pilot_yaw_rate, pilot_pitch, pilot_roll, yaw, pitch, roll, Pbody, Qbody, Rbody);
    

#ifdef DEBUG              
            Serial.print("roll_integral ");
            Serial.print(roll_integral);
            Serial.print("\t");
            Serial.print("Rbody ");
            Serial.print(Rbody);
            Serial.print("\t");
            Serial.print("pitch_integral ");
            Serial.println(pitch_integral);
//            Serial.print("\t");
//            Serial.print("throt_cmd ");
//            Serial.println(throt_cmd);
//            Serial.print("\t");
//            Serial.print("flat_throt_cmd ");
//            Serial.print(flat_throt_cmd);
//            Serial.print("\t");
//            Serial.print("pitch ");
//            Serial.print(pitch);
//            Serial.print("\t");
//            Serial.print("roll ");
//            Serial.print(roll);
//            Serial.print("\t");
//            Serial.print("yaw ");
//            Serial.print(yaw);
//            Serial.print("\t");
//            Serial.print("Pbody ");
//            Serial.print(Pbody);
//            Serial.print("\t");
//            Serial.print("Qbody ");
//            Serial.print(Qbody);
//            Serial.print("\t");
//            Serial.print("Rbody ");
//            Serial.println(Rbody);
//            Serial.print("\t");
//            Serial.print("motor_prime ");
//            Serial.print(motor_prime);
//            Serial.print("\t");
//            Serial.print("prime_check ");
//            Serial.print(prime_check[0]);
//            Serial.print(prime_check[1]);
//            Serial.print("\t");
//            Serial.print("signal_lost ");
//            Serial.print(signal_lost);
//            Serial.print("\t");
//            Serial.print("time ");
//            Serial.print(abs(millis() - time_no_signal));
//            Serial.print("\t");
//            Serial.print("count_no_signal ");
//            Serial.print(count_no_signal);
//            Serial.print("\t");
//            Serial.print("cal_check ");
//            Serial.println(cal_check);


#endif       
    
    //Safety Check - MOTOR OFF/NO THROTTLE
    if (motor_prime == false){  
     
      LH_F_ESC.writeMicroseconds(0);
      RH_F_ESC.writeMicroseconds(0);
      LH_R_ESC.writeMicroseconds(0);
      RH_R_ESC.writeMicroseconds(0);
    }
    else if (cal_check == true){
      
      //Inputs for Throttle Lowpoint - ESC Calibration
      LH_F_ESC.writeMicroseconds(throt_cmd);
      RH_F_ESC.writeMicroseconds(throt_cmd);
      LH_R_ESC.writeMicroseconds(throt_cmd);
      RH_R_ESC.writeMicroseconds(throt_cmd);
      
    }
    else if ((motor_prime == true) && (throt_cmd < 1150) && (signal_lost == false)){  
     
      LH_F_ESC.writeMicroseconds(0);
      RH_F_ESC.writeMicroseconds(0);
      LH_R_ESC.writeMicroseconds(0);
      RH_R_ESC.writeMicroseconds(0);
      motor_prime = false;
      
    }
    else {

      //Positive Yaw Left, Positive Pitch Forward, Positive Roll Left
      LH_F_ESC.writeMicroseconds(flat_throt_cmd - (pitch_input) - (roll_input) + (yaw_input));
      RH_F_ESC.writeMicroseconds(flat_throt_cmd - (pitch_input) + (roll_input) - (yaw_input));
      LH_R_ESC.writeMicroseconds(flat_throt_cmd + (pitch_input) - (roll_input) - (yaw_input));
      RH_R_ESC.writeMicroseconds(flat_throt_cmd + (pitch_input) + (roll_input) + (yaw_input));
    }
 
}
  
void FLT_CNTRL::BEGIN(){

  LH_F_ESC.attach(esc_lh_f_pin, 1000, 2000);
  RH_F_ESC.attach(esc_rh_f_pin, 1000, 2000);
  LH_R_ESC.attach(esc_lh_r_pin, 1000, 2000);
  RH_R_ESC.attach(esc_rh_r_pin, 1000, 2000);
    
  }
  
