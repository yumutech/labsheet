#include "encoders.h"
#include "pid.h"
#define LOOP_DELAY 100

//Pin definitions for motor
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

float Kp_pose = 0; //Proportional gain for position controller
float Kd_pose = 0; //Derivative gain for position controller
float Ki_pose = 0; //Integral gain for position controller
PID leftPose(Kp_pose, Kd_pose, Ki_pose); //Position controller for left wheel position


float target = 0; //Target encoder count

#define BAUD_RATE = 115200;

void setupMotorPins()
{
    // Set our motor driver pins as outputs.
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  // Set initial direction for l and r
  // Which of these is foward, or backward?
  digitalWrite( L_DIR_PIN, LOW  );
  digitalWrite( R_DIR_PIN, LOW );
}

// put your setup code here, to run once:
void setup() 
{

  //Assign motor pins and set direction
  
  // These two function set up the pin
  // change interrupts for the encoders.
  // If you want to know more, find them
  // at the end of this file.  
  setupEncoder0();
  setupEncoder1();


  // Initialise the Serial communication
  // so that we can inspect the values of
  // our encoder using the Monitor.
  Serial.begin( 9600 );
}

void loop() 
{
  
  float output = leftPose.update(target, count_e1);
  Serial.print("Left wheel output is: ");
  Serial.println(output);
  delay(LOOP_DELAY);

}




