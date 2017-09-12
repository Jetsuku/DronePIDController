#include "IMU_sense.h"
#include "flight_controller.h"
#include "PinChangeInterrupt.h"

// Pin Change Interrupt Pins
#define roll_pc_pin 23  //Nano Pin D7
#define pitch_pc_pin 0  //Nano Pin D8
#define yaw_pc_pin 3    //Nano Pin D11
#define throt_pc_pin 4  //Nano Pin D12

//Receiver Arduino Nano Pin
#define roll_pin 7
#define pitch_pin 8
#define yaw_pin 11
#define throt_pin 12

//Variable Delcaration for Interrupt
static int timer_throt_rise;
static int timer_throt_fall;
static int timer_pitch_rise;
static int timer_pitch_fall;
static int timer_yaw_rise;
static int timer_yaw_fall;
static int timer_roll_rise;
static int timer_roll_fall;
volatile static int throt_cmd = 0;
volatile static int pitch_cmd = 0;
volatile static int yaw_cmd = 0;
volatile static int roll_cmd = 0;
static bool cmd_received[3] = {false};

IMU_Sense imu_sense;
FLT_CNTRL flt_cntrl;


///////***Interrupt Functions for Receiver***////////
void InterruptBegin() {

   pinMode(throt_pin, INPUT_PULLUP);
   pinMode(yaw_pin, INPUT_PULLUP);
   pinMode(pitch_pin, INPUT_PULLUP);
   pinMode(pitch_pin, INPUT_PULLUP);
   attachPCINT(throt_pc_pin, Interrupt_Throt, CHANGE);
   attachPCINT(roll_pc_pin, Interrupt_Roll, CHANGE);
   attachPCINT(yaw_pc_pin, Interrupt_Yaw, CHANGE);
   attachPCINT(pitch_pc_pin, Interrupt_Pitch, CHANGE);
  
}


void Interrupt_Throt() {

    uint8_t trigger_throt = getPinChangeInterruptTrigger(throt_pc_pin);
    if(trigger_throt == RISING){
      timer_throt_rise = micros();
       }
    else if(trigger_throt == FALLING){
      timer_throt_fall = micros();
      throt_cmd = timer_throt_fall - timer_throt_rise;
      }
    else{
      }
}

void Interrupt_Yaw() {

    uint8_t trigger_yaw = getPinChangeInterruptTrigger(yaw_pc_pin);
    if(trigger_yaw == RISING){
      timer_yaw_rise = micros();
      cmd_received[0] = true;
      }
    else if(trigger_yaw == FALLING){
      timer_yaw_fall = micros();
      yaw_cmd = timer_yaw_fall - timer_yaw_rise;
      cmd_received[0] = true;
    }
    else{
      cmd_received[0] = false;
      } 
}


void Interrupt_Pitch() {
        
    uint8_t trigger_pitch = getPinChangeInterruptTrigger(pitch_pc_pin);
    if(trigger_pitch == RISING){
      timer_pitch_rise = micros();
      cmd_received[1] = true;
      }
    else if(trigger_pitch == FALLING){
      timer_pitch_fall = micros();
      pitch_cmd = timer_pitch_fall - timer_pitch_rise;
      cmd_received[1] = true;
      }
    else{
      cmd_received[1] = false;
      }
}


void Interrupt_Roll() {

    uint8_t trigger_roll = getPinChangeInterruptTrigger(roll_pc_pin);
    if(trigger_roll == RISING){
      timer_roll_rise = micros();
      cmd_received[2] = true;
      }
    else if(trigger_roll == FALLING){
      timer_roll_fall = micros();
      roll_cmd = timer_roll_fall - timer_roll_rise;
      cmd_received[2] = true;
      }
    else{
      cmd_received[2] = false;
      }
}
///////***End Interrupt Functions for Receiver***///////


void setup() {
////
//  Serial.begin(115200);
  flt_cntrl.BEGIN();
  InterruptBegin();
  imu_sense.Begin();
  flt_cntrl.CAL_ESC(throt_cmd, pitch_cmd);
  flt_cntrl.CAL_REC(pitch_cmd, yaw_cmd, roll_cmd);


}

void loop() {
  
  imu_sense.Receive();
  flt_cntrl.CMD(throt_cmd, pitch_cmd, yaw_cmd, roll_cmd, cmd_received);
  
  cmd_received[0] = false;
  cmd_received[1] = false;
  cmd_received[2] = false;
      
}



