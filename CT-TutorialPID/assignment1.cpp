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

#include "assignment1.h"

bool Robot::init() {
  MECOtron::init(); // Initialize the MECOtron

  // Initializing the robot's specific variables
  startTime = millis();

  return true;
}

void Robot::control() {

  // Compute update of motor voltages if controller is enabled (triggered by
  // pushing 'Button 0' in QRoboticsCenter)
  if(controlEnabled()) {
    // Fill your control law here to conditionally update the motor voltage...
    unsigned long elapsed = millis() - startTime;
    unsigned long cycleTime = elapsed % 14000;  // 14 second cycle that repeats
    float voltage = 0.0;
    
    if (cycleTime < 2000) {
      voltage = 4.0;   // 0-2s: +4V
    } else if (cycleTime < 3000) {
      voltage = 0.0;   // 2-3s: 0V
    } else if (cycleTime < 5000) {
      voltage = 2.0;   // 3-5s: +2V
    } else if (cycleTime < 7000) {
      voltage = 0.0;   // 5-7s: 0V
    } else if (cycleTime < 9000) {
      voltage = -4.0;  // 7-9s: -4V
    } else if (cycleTime < 10000) {
      voltage = 0.0;   // 9-10s: 0V
    } else if (cycleTime < 12000) {
      voltage = -2.0;  // 10-12s: -2V
    } else {
      voltage = 0.0;   // 12-14s: 0V (2s wait before cycle restarts)
    }

    LED1(ON);
    LED2(OFF);
    writeValue(3, voltage);

    setVoltageMotorA(voltage);
    setVoltageMotorB(voltage);
  } else {
    // If the controller is disabled, you might want to do something else...
    LED1(OFF);
    LED2(ON);
    setVoltageMotorA(0.0); // Apply 0.0 volts to motor A if the control is disabled
    setVoltageMotorB(0.0); // Apply 0.0 volts to motor B if the control is disabled
  }

//GET POSITION 
 // float encA_value = getPositionMotorA();
 // writeValue(3, encA_value); // write the encoder value to channel 3

 // float frontDISTANCE = getFrontDistance();
 // writeValue(4, frontDISTANCE); // write the encoder value to channel 3


  float va = getSpeedMotorA();    // Get the wheel speed of motor A (in radians/second)
  writeValue(1, va);
  float vb = getSpeedMotorB();
  writeValue(2, vb);
  
  
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
