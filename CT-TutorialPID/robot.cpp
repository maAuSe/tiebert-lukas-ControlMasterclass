/*
 * MECOTRON TUTORIAL
 *
 * This is a template to get you started in the course of the tutorial on the
 * control theory platforms, a.k.a. the MECOtrons.s
 * The tasks of the tutorial session will guide you through this template and
 * ask you to make use of the platform's capabilities in a step-by-step fashion.
 *
 * Every function in this template comes with an opening comment that describes
 * its purpose and functionality. Please also pay attention to the remarks that
 * are made in comment blocks.
 *
 */

#include "robot.h"

bool Robot::init() {
  MECOtron::init(); // Initialize the MECOtron

  // Initializing the robot's specific variables
  for(int k=0; k<2; k++){
    x[k] = 0.0; 
    uA[k] = 0.0;
    uB[k] = 0.0;
    eA[k] = 0.0;
    eB[k] = 0.0;  // Set all components of the vector (float array) x to 0 as initialization
  }

  return true;
}

void Robot::control() {

  // Compute update of motor voltages if controller is enabled (triggered by
  // pushing 'Button 0' in QRoboticsCenter)
  if(controlEnabled()) {
    // Fill your control law here to conditionally update the motor voltage...
  float r_A = 2; 
  float r_B = 2; 
  float motor_angle_A = getPositionMotorA();
  float motor_angle_B = getPositionMotorB();
  eA[0] = eA[1];
  eA[1] = r_A - motor_angle_A;
  eB[0] = eB[1];
  eB[1] = r_B - motor_angle_B;
  uA[0]=uA[1];
  uB[0]=uB[1];

  uA[1]=2.05*eA[1]-1.95*eA[0]+uA[0];
  uB[1]=2.05*eB[1]-1.95*eB[0]+uB[0];



    LED1(ON);
    LED2(OFF);
    setVoltageMotorA(uA[1]);
    setVoltageMotorB(uB[1]);

   // setVoltageMotorA(-12.0); // Apply 6.0 volts to motor A if the control is enabled
   // setVoltageMotorB(-12.0); // Apply 2.0 volts to motor B if the control is enabled
  } else {
    // If the controller is disabled, you might want to do something else...
    LED1(OFF);
    LED2(ON);
    setVoltageMotorA(0.0); // Apply 0.0 volts to motor A if the control is disabled
    setVoltageMotorB(0.0); // Apply 0.0 volts to motor B if the control is disabled
  }

//GET POSITION 
 float encA_value = getPositionMotorA();
 writeValue(3, encA_value); // write the encoder value to channel 3

 float frontDISTANCE = getFrontDistance();
 writeValue(4, frontDISTANCE); // write the encoder value to channel 3


  float va = getSpeedMotorA();    // Get the wheel speed of motor A (in radians/second)
  x[1] = x[0]; x[0] = va;         // Memorize the last two samples of the speed of motor A (in fact, a shift register)

  float k = readValue(0); // Read the value you set on QRoboticsCenter's channel 0
  writeValue(0, k);       // Send the value of variable k to QRoboticsCenter's channel 0

}

bool Robot::controlEnabled() {
  return _button_states[0];       // The control is enabled if the state of button 0 is true
}

void Robot::button0callback() {
  if(toggleButton(0)) {           // Switches the state of button 0 and checks if the new state is true
    message("Robot enabled.");    // Display a message in the status bar of QRoboticsCenter
  }
  else {
    message("Robot disabled.");
  }
}

void Robot::button1callback() {
  toggleButton(1);
  init();                         // Reset the MECOtron and reinitialize the Robot object
  message("Reset.");
}
